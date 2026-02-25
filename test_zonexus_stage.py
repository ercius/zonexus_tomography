# -*- coding: utf-8 -*-
"""
Interactive test script for Zonexus_Continuous_Rotation_Stage_Control.
Run this standalone to test the holder without the microscope or server.
"""

import argparse
from zonexus_stage import Zonexus_Continuous_Rotation_Stage_Control


def main(zncom):
    print(f'Connecting to Zonexus stage on {zncom}...')
    stage = Zonexus_Continuous_Rotation_Stage_Control(zncom)
    print('Connected.')

    while True:
        print('\nCommands:')
        print('  g        - get current alpha')
        print('  s <deg>  - set alpha to <deg> degrees')
        print('  p        - park motor')
        print('  u        - unpark motor')
        print('  x        - stop motor')
        print('  q        - quit')

        cmd = input('> ').strip().split()
        if not cmd:
            continue

        if cmd[0] == 'q':
            break
        elif cmd[0] == 'g':
            alpha = stage.getAlpha()
            print(f'Alpha = {alpha:.4f} deg')
        elif cmd[0] == 's':
            if len(cmd) < 2:
                print('Usage: s <degrees>')
            else:
                target = float(cmd[1])
                print(f'Moving to {target:.4f} deg...')
                stage.setAlpha(target)
                alpha = stage.getAlpha()
                print(f'At {alpha:.4f} deg')
        elif cmd[0] == 'p':
            stage.parkMotor(None)
            print('Motor parked')
        elif cmd[0] == 'u':
            stage.unparkMotor(None)
            print('Motor unparked')
        elif cmd[0] == 'x':
            stage.stopMotor(None)
            print('Motor stopped')
        else:
            print(f'Unknown command: {cmd[0]}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--com', default='COM3', help='Serial port for Zonexus stage')
    args = parser.parse_args()

    main(args.com)
