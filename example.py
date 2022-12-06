from stupidArtnet import StupidArtnet, StupidArtnetServer
import time
import random, colorsys
import numpy as np



# THESE ARE MOST LIKELY THE VALUES YOU WILL BE NEEDING
num_trees = 2
ips = [237,]
ips = [f'10.0.0.{ip}' for ip in ips]
sensors = [0 for ip in ips]
# target_ip = '10.0.0.24'		# typically in 2.x or 10.x range
universe = 0 										# see docs
packet_size = 90								# it is not necessary to send whole universe

def test_callback(data):
    """Test function to receive callback data."""
    # the received data is an array
    # of the channels value (no headers)
    source = data[0]
    if data[1] > 0:
        sensors[source] = data[1]
    print(f'Rx from {source}', data)

frame_rate = 10
frame_rate_update = 100
artnets_tx = [StupidArtnet(ip, universe, packet_size, frame_rate, True, False) for ip in ips]
artnet_rx = StupidArtnetServer()
listener = artnet_rx.register_listener(0, callback_function=test_callback)
print('listening')


# CHECK INIT
print([a for a in artnets_tx])

transmit = 1

if transmit:
    for a in artnets_tx:
        a.start()  # start continuous sending

    packet = bytearray(packet_size)
    # AND MODIFY THE DATA AS YOU GO
    # t = time.time()
    print('updating colors')
    for fi in range(frame_rate_update * 30):
        for node, artnet in enumerate(artnets_tx):
            for pixel in range(int(packet_size/3)):  	# Fill buffer with random stuff

                hue = np.clip(sensors[node] / 300, 0, 1)
                rgb = colorsys.hsv_to_rgb(hue, 1, 1)
                print(rgb)
                for ci in range(3):
                    packet[pixel * 3 + ci] = int(rgb[ci] * 255)
            artnet.set(packet)
            artnet.show()

            # print(a.buffer)
        time.sleep(1/frame_rate_update)

    # SOME DEVICES WOULD HOLD LAST DATA, TURN ALL OFF WHEN DONE
    for a in artnets_tx:
        a.blackout()
        a.stop()

    # ... REMEMBER TO CLOSE THE THREAD ONCE YOU ARE DONE

    # CLEANUP IN THE END
    for a in artnets_tx:
        del a

else:
    time.sleep(10)

print('done')