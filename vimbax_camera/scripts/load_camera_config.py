from vmbpy import *

import xml.etree.ElementTree as ET

import argparse
import os   
import time
from contextlib import ExitStack
from typing import Optional

PTP_MODES = {'off': 'Off', 'auto': 'Auto', 'slave': 'Slave'}


def abort(msg):
    print(f'{msg}')
    exit(1)

def extract_camera_id_from_xml(file_path):
    try:
        # Parse the XML (CML) file
        tree = ET.parse(file_path)
        root = tree.getroot()

        # Find the 'CameraInfo' element and extract its 'Id' attribute
        camera_info = root.find('CameraInfo')
        if camera_info is not None and 'Id' in camera_info.attrib:
            camera_id = camera_info.attrib['Id']
            print(f"Camera Info Id: {camera_id}")
            return camera_id
        else:
            print("No CameraInfo or Id attribute found in the file.")

    except ET.ParseError as e:
        print(f"Error parsing CML file: {e}")
    except FileNotFoundError:
        print(f"File not found: {file_path}")

       
def main():
    parser = argparse.ArgumentParser(description='Load camera configuration')
    parser.add_argument('--settings_folder', type=str, default='/path/to/settings/folder', help='Folder to save/load camera settings')
    parser.add_argument('--trigger', choices=['action', 'fixed_rate'], default='action')
    parser.add_argument('--action-keys', type=int, nargs=3, default=[1, 1, 1], metavar=('DEVICE', 'GROUP', 'MASK'))
    parser.add_argument('--trigger-rate-hz', type=float, default=30.0)
    parser.add_argument('--exposure-auto-max-us', type=int, default=20000)
    parser.add_argument('--ptp', choices=['off', 'auto', 'slave'], default='slave')
    parser.add_argument('--ptp-timeout-s', type=float, default=30.0)
    parser.add_argument('--min-packet-size', type=int, default=8000)
    args = parser.parse_args()

    # Check if the settings folder exists.
    if not os.path.exists(args.settings_folder):
        abort('Given settings folder does not exist')

    settings = {}
    for file in os.listdir(args.settings_folder):
        if file.endswith('.xml'):
            print(f'Loading settings from {file}')
            settings_file = os.path.join(args.settings_folder, file)
            camera_id = extract_camera_id_from_xml(settings_file)
            settings[camera_id] = settings_file

    with VmbSystem.get_instance() as vmb:
        cams = vmb.get_all_cameras()
        missing = sorted(set(settings) - {cam.get_id() for cam in cams})
        if missing:
            abort(f'Cameras with settings in {args.settings_folder} not connected: {", ".join(missing)}')
        for cam in cams:
            with cam:
                # Restore settings to initial value.
                try:
                    cam.UserSetSelector.set('Default')

                except (AttributeError, VmbFeatureError):
                    print('Failed to set Feature \'UserSetSelector\'')

                try:
                    cam.UserSetLoad.run()
                    print("--> All feature values have been restored to default")

                except (AttributeError, VmbFeatureError):
                    abort('Failed to run Feature \'UserSetLoad\'')

                cam_id = cam.get_id()

                if cam_id in settings.keys():
                    print(f'Loading settings for camera {cam_id}')
                    cam.load_settings(settings[cam_id], PersistType.All)
                else:
                    print(f'No settings found for camera {cam_id}')

                stream = cam.get_streams()[0]
                stream.GVSPAdjustPacketSize.run()
                while not stream.GVSPAdjustPacketSize.is_done():
                    time.sleep(0.05)
                packet_size = stream.GVSPPacketSize.get()
                print(f'{cam_id}: GVSPPacketSize={packet_size}')
                if packet_size < args.min_packet_size:
                    abort(f'Camera {cam_id}: packet size {packet_size} is below {args.min_packet_size}; '
                          f'jumbo frames are not getting through (host MTU 9000 needed)')

                if args.trigger == 'action':
                    device_key, group_key, group_mask = args.action_keys
                    cam.TriggerSelector.set('FrameStart')
                    cam.TriggerSource.set('Action0')
                    cam.TriggerMode.set('On')
                    cam.ActionDeviceKey.set(device_key)
                    cam.ActionSelector.set(0)
                    cam.ActionGroupKey.set(group_key)
                    cam.ActionGroupMask.set(group_mask)

                cam.ExposureAuto.set('Continuous')
                cam.GainAuto.set('Continuous')
                cam.ExposureAutoMax.set(args.exposure_auto_max_us)

                exposure_auto = cam.ExposureAuto.get()
                exposure_time = cam.ExposureTimeAbs.get()
                cam.ExposureAuto.set('Off')
                cam.ExposureTimeAbs.set(args.exposure_auto_max_us)
                frame_rate_limit = cam.AcquisitionFrameRateLimit.get()
                cam.ExposureTimeAbs.set(exposure_time)
                cam.ExposureAuto.set(exposure_auto)

                if frame_rate_limit < args.trigger_rate_hz * 1.02:
                    abort(f'Camera {cam_id}: AcquisitionFrameRateLimit {frame_rate_limit:.2f} fps at exposure cap '
                          f'{args.exposure_auto_max_us} us is below trigger rate {args.trigger_rate_hz} Hz + 2% '
                          f'(StreamBytesPerSecond {cam.StreamBytesPerSecond.get()}, '
                          f'max {cam.StreamBytesPerSecond.get_range()[1]})')

                cam.PtpMode.set(PTP_MODES[args.ptp])

        with ExitStack() as stack:
            for cam in cams:
                stack.enter_context(cam)

            if args.ptp != 'off':
                deadline = time.monotonic() + args.ptp_timeout_s
                while True:
                    ptp_statuses = {cam.get_id(): str(cam.PtpStatus.get()) for cam in cams}
                    statuses = list(ptp_statuses.values())
                    masters_expected = 1 if args.ptp == 'auto' else 0
                    if (statuses.count('Master') == masters_expected and
                            statuses.count('Slave') == len(statuses) - masters_expected):
                        print(f'PTP locked: {ptp_statuses}')
                        break
                    if time.monotonic() > deadline:
                        abort(f'PTP did not lock within {args.ptp_timeout_s} s: {ptp_statuses} '
                              f'(is ptp4l-cameras.service running?)')
                    time.sleep(0.5)

            for cam in cams:
                print(f'{cam.get_id()}: TriggerSource={cam.TriggerSource.get()} '
                      f'TriggerMode={cam.TriggerMode.get()} '
                      f'ExposureAutoMax={cam.ExposureAutoMax.get()} '
                      f'AcquisitionFrameRateLimit={cam.AcquisitionFrameRateLimit.get():.2f} '
                      f'ExposureAuto={cam.ExposureAuto.get()} GainAuto={cam.GainAuto.get()} '
                      f'PtpMode={cam.PtpMode.get()} PtpStatus={cam.PtpStatus.get()}')

if __name__ == '__main__':
    main()