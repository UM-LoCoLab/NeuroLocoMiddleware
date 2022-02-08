from sys import path
try:
  from SoftRealtimeLoop import SoftRealtimeLoop
except ModuleNotFoundError:
  print("module SoftRealtimeLoop not found in path: %s"%path)
  print("to add SoftRealtimeLoop to the path, edit the .bashrc file like so:")
  print("""
export PYTHONPATH={$PYTHONPATH}:/home/pi/NeuroLocoMiddleware.
    """)
  path.append("/home/pi/NeuroLocoMiddleware")
  from SoftRealtimeLoop import SoftRealtimeLoop
from ActPacMan import ActPacMan


def current_control_demo():
    hold_current=[1000]
    time=6
    time_step=0.1

    with ActPacMan('/dev/ttyACM0') as dev:
        # dev.__enter__()
            # dev_id = fxs.open(port, baud_rate, log_level=6)
            # fxs.start_streaming(dev_id, 100, log_en=False)
            # app_type = fxs.get_app_type(dev_id)


        print("Setting controller to current...")
        dev.set_current_gains(kp=40, ki=400, ff=128)
        assert(dev._state == _ActPacManStates.CURRENT)
            # # Gains are, in order: kp, ki, kd, K, B & ff
            # # dev.set_gains(kp=40, ki=400, ff=128)
            # fxs.set_gains(dev_id, 40, 400, 0, 0, 0, 128)

        # sleep(0.5) # can we delete this?

        prev_current = hold_current[0]
        num_time_steps = int(time / time_step)

        for current in hold_current:
            for i in range(num_time_steps):
                des_current = int(
                    (current - prev_current) * (i / float(num_time_steps)) + prev_current
                )

                try:
                    dev.ϕ = 0.0
                except RuntimeError as e:
                    pass
                else:
                    raise AssertionError()
                dev.update()

                dev.i = (des_current/1000.) # q-axis current in Amps
                # fxs.send_motor_command(dev_id, fxe.FX_CURRENT, des_current)
                sleep(time_step)
                # fxu.clear_terminal()
                # print("Desired (A):         ", 1000*des_current)
                # print("Measured (A):        ", dev.i)
                # print("Difference (A):      ", (dev.i - 1000*des_current), "\n")

                # # dev.dephy_print_device()
                # fxu.print_device(dev.act_pack, dev.app_type)
            prev_current = current

        print("Turning off current control...")
        # Ramp down first
        ramp_down_steps = 50
        for step in range(ramp_down_steps):
            des_current = prev_current * (ramp_down_steps - step) / ramp_down_steps
            dev.i=(des_current/1000.) # current in Amps
            dev.update()
            # fxu.clear_terminal()
            # print("Desired (A):         ", 1000*des_current)
            # print("Measured (A):        ", dev.get_current_qaxis_amps())
            # print("Difference (A):      ", (dev.get_current_qaxis_amps() - 1000*des_current), "\n")
            # # dev.dephy_print_device()
            # fxu.print_device(dev.act_pack, dev.app_type)
            sleep(time_step)
    

if __name__ == '__main__':
    from time import sleep
    from SoftRealtimeLoop import SoftRealtimeLoop # helps with the shutdown process
    current_control_demo()
    print("done with current_control_demo()")