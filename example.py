from stupidArtnet import StupidArtnet, StupidArtnetServer
import time
import random

def test_callback(data):
    """Test function to receive callback data."""
    # the received data is an array
    # of the channels value (no headers)
    print('Received new data \n', data)

# THESE ARE MOST LIKELY THE VALUES YOU WILL BE NEEDING
num_trees = 2
ips = [24, 236, *[1 for r in range(1)]]
ips = [f'10.0.0.{ip}' for ip in ips]
# target_ip = '10.0.0.24'		# typically in 2.x or 10.x range
universe = 0 										# see docs
packet_size = 90								# it is not necessary to send whole universe

# CREATING A STUPID ARTNET OBJECT
# SETUP NEEDS A FEW ELEMENTS
# TARGET_IP   = DEFAULT 127.0.0.1
# UNIVERSE    = DEFAULT 0
# PACKET_SIZE = DEFAULT 512
# FRAME_RATE  = DEFAULT 30
# ISBROADCAST = DEFAULT FALSE
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

        for a in artnets_tx:
            for pixel in range(int(packet_size/3)):  	# Fill buffer with random stuff
                for color in range(3):
                    packet[pixel * 3 + color] = random.randint(0, 100)
            a.set(packet)
            a.show()

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