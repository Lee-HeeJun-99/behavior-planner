# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import threading
import time

import serial

import rclpy

from libnmea_navsat_driver.driver import Ros2NMEADriver

# [K-ROAD] RTK 보정(RTCM) 입력 / GGA 출력 지원 (ntrip_client 연동)
#   /rtcm (rtcm_msgs/Message)  -> 같은 시리얼(USB) 포트로 수신기에 write
#   GGA 문장 -> /nmea (nmea_msgs/Sentence) -> ntrip_client 가 보정 서버로 전송
try:
    from rtcm_msgs.msg import Message as RtcmMsg
except ImportError:
    RtcmMsg = None
try:
    from nmea_msgs.msg import Sentence
except ImportError:
    Sentence = None


def main(args=None):
    rclpy.init(args=args)

    driver = Ros2NMEADriver()
    frame_id = driver.get_frame_id()

    # serial_port = driver.declare_parameter('port', '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0').value

    serial_port = driver.declare_parameter('port', '/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTD4CZMY-if00-port0').value
    serial_baud = driver.declare_parameter('baud', 115200).value
    enable_rtcm = driver.declare_parameter('enable_rtcm', True).value           # /rtcm -> 수신기
    rtcm_topic = driver.declare_parameter('rtcm_topic', 'rtcm').value
    publish_gga = driver.declare_parameter('publish_gga', True).value           # GGA -> /nmea
    nmea_topic = driver.declare_parameter('nmea_topic', 'nmea').value
    gga_period = driver.declare_parameter('gga_period_sec', 1.0).value          # 보정 서버로 보낼 GGA 주기

    try:
        GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)
        driver.get_logger().info("Successfully connected to {0} at {1}.".format(serial_port, serial_baud))

        write_lock = threading.Lock()
        rtcm_stat = {'bytes': 0, 'msgs': 0}

        if enable_rtcm:
            if RtcmMsg is None:
                driver.get_logger().error("rtcm_msgs not installed -> RTCM input disabled")
            else:
                def rtcm_cb(msg):
                    try:
                        with write_lock:
                            GPS.write(bytes(msg.message))
                        rtcm_stat['bytes'] += len(msg.message)
                        rtcm_stat['msgs'] += 1
                    except Exception as e:  # noqa: B902
                        driver.get_logger().warn("RTCM write failed: {0}".format(e))
                driver.create_subscription(RtcmMsg, rtcm_topic, rtcm_cb, 10)
                driver.get_logger().info("RTCM input enabled: /{0} -> {1}".format(rtcm_topic, serial_port))

                def rtcm_report():
                    driver.get_logger().info("RTCM -> receiver: {0} msgs, {1} bytes (last 10 s)".format(
                        rtcm_stat['msgs'], rtcm_stat['bytes']))
                    rtcm_stat['bytes'] = 0
                    rtcm_stat['msgs'] = 0
                driver.create_timer(10.0, rtcm_report)

        gga_pub = None
        if publish_gga:
            if Sentence is None:
                driver.get_logger().error("nmea_msgs not installed -> GGA output disabled")
            else:
                gga_pub = driver.create_publisher(Sentence, nmea_topic, 10)
        last_gga = [0.0]

        # 구독 콜백(RTCM)은 별도 스레드에서 처리, 메인 스레드는 기존처럼 시리얼 읽기
        spin_thread = threading.Thread(target=rclpy.spin, args=(driver,), daemon=True)
        spin_thread.start()

        try:
            while rclpy.ok():
                data = GPS.readline().strip()
                try:
                    if isinstance(data, bytes):
                        data = data.decode("utf-8")
                    driver.add_sentence(data, frame_id)
                    if gga_pub is not None and data[3:6] == 'GGA':
                        now = time.monotonic()
                        if now - last_gga[0] >= gga_period:
                            last_gga[0] = now
                            m = Sentence()
                            m.header.stamp = driver.get_clock().now().to_msg()
                            m.header.frame_id = frame_id
                            m.sentence = data
                            gga_pub.publish(m)
                except ValueError as e:
                    driver.get_logger().warn(
                        "Value error, likely due to missing fields in the NMEA message. Error was: %s. "
                        "Please report this issue at github.com/ros-drivers/nmea_navsat_driver, including a bag file "
                        "with the NMEA sentences that caused it." % e)

        except Exception as e:
            driver.get_logger().error("Ros error: {0}".format(e))
            GPS.close()  # Close GPS serial port
    except serial.SerialException as ex:
        driver.get_logger().fatal("Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))
