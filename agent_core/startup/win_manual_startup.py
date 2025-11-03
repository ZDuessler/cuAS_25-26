import subprocess

if False:  # external mavproxy

    # For every simulator MissionPlanner starts, add that as an input port here
    input_ports = [
        'tcp:127.0.0.1:5760',
        'tcp:127.0.0.1:5770',
        # 'tcp:127.0.0.1:5780'
    ]

    # For each output port MavProxy should produce, place that here
    output_ports = [
        'udpin:127.0.0.1:14101',
        'udp:127.0.0.1:14551',
        'udp:127.0.0.1:14552',
        # 'udpout:127.0.0.1:14103'
    ]

    mavproxy_str = 'start /wait mavproxy'
    for input_port in input_ports:
        mavproxy_str += str(" --master " + input_port)
    for output_port in output_ports:
        mavproxy_str += str(" --out " + output_port)
    mavproxy_str += str(" --streamrate=-1")
    mavproxy_str += str(" --console")

    # This will open a new terminal and run the command (e.g., 'python bb.py')
    print(mavproxy_str)
    subprocess.call(mavproxy_str, shell=True)

else:  # local mavproxy

    drone_count = 2
    local_mavproxy = []

    for x in range(0, drone_count):
        local_mavproxy.append(
            {
                'input': [
                    # f'tcp:127.0.0.1:5{760 + 10*x}'
                    # f'tcp:10.20.0.40:5{760 + 10*x}'
                    # f'tcp:10.0.0.31:5{760 + 10*x}'
                    f'udpout:10.0.0.31:145{51 + x}'
                ],
                'output': [
                    f'udp:127.0.0.1:145{51+x}',
                    f'udpin:127.0.0.1:141{str(x+1).zfill(2)}'
                ]
            }
        )

    # --home=39.01876523734408,-104.8938077760428
    # --home=39.01814, -104.8924

    for mavproxy in local_mavproxy:
        mavproxy_str = 'start /wait mavproxy'
        for input_port in mavproxy['input']:
            mavproxy_str += str(" --master " + input_port)
        for output_port in mavproxy['output']:
            mavproxy_str += str(" --out " + output_port)
        mavproxy_str += str(" --streamrate=-1")
        # mavproxy_str += str(" --console")

        # This will open a new terminal and run the command
        # (e.g., 'python bb.py')
        print(mavproxy_str)
        subprocess.Popen(mavproxy_str, shell=True)

# ###########################################################