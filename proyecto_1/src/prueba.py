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

directory = "proyecto_1/audio"
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

# The maximum frequency is half the sample rate
points_per_freq = len(xf) / (SAMPLE_RATE / 2)

# Our target frequency is 4000 Hz
target_idx = int(points_per_freq * 4000)

yf[target_idx - 1 : target_idx + 2] = 0

plt.plot(xf, numpy.abs(yf))
plt.show()


new_sig = scipy.fft.irfft(yf)

plt.plot(new_sig[:1000])
plt.show()

norm_new_sig = numpy.int16(new_sig * (32767 / new_sig.max()))

scipy.io.wavfile.write("clean.wav", SAMPLE_RATE, norm_new_sig)