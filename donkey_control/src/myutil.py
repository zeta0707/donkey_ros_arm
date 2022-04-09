import time

def clamp(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n

def clampRem(n, minn, maxn):
    if n < minn:
        #print(n, minn)
        return (n - minn) , minn
    elif n > maxn:
        #print(n, maxn)
        return (n - maxn), maxn
    else:
        return 0, n

class PCA9685:
    """
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    """

    def __init__(
           self, channel, address, frequency=60, busnum=None, init_delay=0.1
    ):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        import Adafruit_PCA9685

        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C

            # replace the get_bus function with our own
            def get_bus():
                return busnum

            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        time.sleep(init_delay)  # "Tamiya TBLE-02" makes a little leap otherwise

        self.pulse = 380
        self.running = True

    def set_pwm(self, pulse):
        try:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))
        except:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))

    def run(self, pulse):
        self.set_pwm(pulse)

    def set_pwm_clear(self):
        self.pwm.set_all_pwm(0,0)    

    def runTarget(self, pulseSta, pulseTar):        
        if pulseTar > pulseSta:
            step = 4
        else:
            step = -4
        self.set_pwm(pulseSta) 
        for tempPulse in range(pulseSta, pulseTar, step):
            #print(tempPulse)
            self.set_pwm(tempPulse)              
            time.sleep(0.02)
        #print("Done")

    def set_pulse(self, pulse):
        self.pulse = pulse

    def update(self):
        while self.running:
            self.set_pulse(self.pulse)        