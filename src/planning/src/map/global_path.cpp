#include "planning_pkg_2025/map/global_path.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <utility>

namespace planning
{
bool GlobalPath::load(const std::string & file_path, std::string * error)
{
  return load(file_path, error, 0.75);
}

bool GlobalPath::load(const std::string & file_path, std::string * error, double smoothing_distance)
{
  std::ifstream input(file_path);
  if (!input) {
    if (error) {*error = "cannot open global path: " + file_path;}
    return false;
  }
  Path loaded;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') {continue;}
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);
    PathPoint point;
    if (!(values >> point.x >> point.y)) {
      if (error) {*error = "invalid global path line " + std::to_string(line_number);}
      return false;
    }
    values >> point.yaw >> point.curvature;
    loaded.push_back(point);
  }
  if (loaded.size() < 2) {
    if (error) {*error = "global path needs at least two points";}
    return false;
  }
  if (closed_) {
    if (std::hypot(loaded.front().x-loaded.back().x,loaded.front().y-loaded.back().y)<1e-6) loaded.pop_back();
    if (loaded.size()<3 || std::hypot(loaded.front().x-loaded.back().x,loaded.front().y-loaded.back().y)>0.25) {
      if(error) *error="closed route requires >=3 points and an endpoint gap <=0.25m";
      return false;
    }
  }
  // Smooth short survey/join discontinuities using distance-weighted coordinates.
  // Preserve endpoints; never rewrite the user's map file.
  if (smoothing_distance > 0.0) {
    const Path original = loaded;
    std::vector<double> arc(loaded.size(), 0.0);
    for (std::size_t i = 1; i < loaded.size(); ++i) {
      arc[i] = arc[i - 1] + std::hypot(original[i].x - original[i - 1].x, original[i].y - original[i - 1].y);
    }
    const double circumference=arc.back()+std::hypot(original.back().x-original.front().x,original.back().y-original.front().y);
    for (std::size_t i = closed_ ? 0 : 1; i < loaded.size()-(closed_ ? 0 : 1); ++i) {
      const double sigma = closed_ ? std::min(smoothing_distance,circumference/6.0) :
        std::min(smoothing_distance, std::min(arc[i], arc.back() - arc[i]) / 3.0);
      if (sigma < 1e-6) {continue;}
      double x = 0.0, y = 0.0, weight = 0.0;
      for(int turn=closed_ ? -1 : 0;turn<=(closed_ ? 1 : 0);++turn){
        const double center=arc[i]-turn*circumference;
        auto begin=std::lower_bound(arc.begin(),arc.end(),center-3.0*sigma);
        auto end=std::upper_bound(arc.begin(),arc.end(),center+3.0*sigma);
        for(auto j=static_cast<std::size_t>(begin-arc.begin());j<static_cast<std::size_t>(end-arc.begin());++j){
          const double d=(arc[j]-center)/sigma;
          const double w=std::exp(-0.5*d*d);
          x+=w*original[j].x;y+=w*original[j].y;weight+=w;
        }
      }
      const double dx = x / weight - original[i].x;
      const double dy = y / weight - original[i].y;
      // Bound changes by 30 cm, independent of the requested smoothing window.
      const double shift = std::hypot(dx, dy);
      const double scale = shift > 0.3 ? 0.3 / shift : 1.0;
      loaded[i].x = original[i].x + dx * scale;
      loaded[i].y = original[i].y + dy * scale;
    }
  }
  for (std::size_t i = 0; i < loaded.size(); ++i) {
    const auto & before = loaded[i > 0 ? i - 1 : (closed_ ? loaded.size()-1 : i)];
    const auto & after = loaded[i + 1 < loaded.size() ? i + 1 : (closed_ ? 0 : i)];
    loaded[i].yaw = std::atan2(after.y - before.y, after.x - before.x);
  }
  for (std::size_t i = closed_ ? 0 : 1; i < loaded.size()-(closed_ ? 0 : 1); ++i) {
    const auto before=(i+loaded.size()-1)%loaded.size();
    const auto after=(i+1)%loaded.size();
    const double ds=std::hypot(loaded[after].x-loaded[before].x,loaded[after].y-loaded[before].y);
    loaded[i].curvature=ds>1e-6 ? std::remainder(loaded[after].yaw-loaded[before].yaw,2.0*M_PI)/ds : 0.0;
  }
  if (!closed_) {
    loaded.front().curvature=loaded[1].curvature;
    loaded.back().curvature=loaded[loaded.size()-2].curvature;
  }
  storePath(std::move(loaded));
  return true;
}

void GlobalPath::storePath(Path path) {
  lap_size_=path.size();path_=std::move(path);
  // Local generation uses 300 samples; obstacle lookup needs another 200.
  if(closed_ && lap_size_>0){path_.reserve(lap_size_+500);for(std::size_t i=0;i<500;++i)path_.push_back(path_[i%lap_size_]);}
}
void GlobalPath::setPath(Path path) {storePath(std::move(path));}

std::size_t GlobalPath::nearestIndex(const Point2d & point, std::size_t hint) const
{
  if (path_.empty()) {return 0;}
  const std::size_t size=lap_size_;
  const bool acquire=hint>=size;
  const std::size_t count=acquire ? size : (closed_ ? std::min<std::size_t>(size,600) : std::min(size,hint+500)-(hint>100 ? hint-100 : 0));
  const std::size_t begin=acquire ? 0 : (closed_ ? (hint+size-std::min<std::size_t>(100,size-1))%size : (hint>100 ? hint-100 : 0));
  std::size_t best=begin;
  double best_squared=std::numeric_limits<double>::infinity();
  for(std::size_t offset=0;offset<count;++offset){
    const std::size_t i=closed_ ? (begin+offset)%size : begin+offset;
    const double dx=path_[i].x-point.x,dy=path_[i].y-point.y;
    const double squared=dx*dx+dy*dy;
    if(squared<best_squared){best_squared=squared;best=i;}
  }
  return best;
}
}  // namespace planning
