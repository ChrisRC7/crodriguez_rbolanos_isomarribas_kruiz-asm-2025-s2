import numpy, scipy, os 
from matplotlib import pyplot as plt



#Referencia: https://realpython.com/python-sine-wave/

SAMPLE_RATE = 44100  # Hertz
DURATION = 5  # Seconds

def generate_sine_wave(freq, sample_rate, duration):
    x = numpy.linspace(0, duration, sample_rate * duration, endpoint=False)
    frequencies = x * freq
    # 2pi because np.sin takes radians
    y = numpy.sin((2 * numpy.pi) * frequencies)
    return x, y

_, nice_tone = generate_sine_wave(400, SAMPLE_RATE, DURATION)
_, noise_tone = generate_sine_wave(4000, SAMPLE_RATE, DURATION)
noise_tone = noise_tone * 0.3

mixed_tone = nice_tone + noise_tone

normalized_tone = numpy.int16((mixed_tone / mixed_tone.max()) * 32767)

directory = "audio"
filename = "mysinewave.wav"

output_path = os.path.join(directory, filename)

# Remember SAMPLE_RATE = 44100 Hz is our playback rate
scipy.io.wavfile.write(output_path, SAMPLE_RATE, normalized_tone)

plt.plot(normalized_tone[:1000])
plt.show()

# Number of samples in normalized_tone
N = SAMPLE_RATE * DURATION

yf =scipy.fft.rfft(normalized_tone)
xf = scipy.fft.rfftfreq(N, 1 / SAMPLE_RATE)

plt.plot(xf, numpy.abs(yf))
plt.show()