import os


# For it to work: sudo apt install sox\
def play_sound(duration=0.08, freq=1000):
    os.system('play -nq -t alsa synth {} sine {}'.format(duration, freq))
