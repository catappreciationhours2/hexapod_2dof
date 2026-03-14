import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(21,GPIO.OUT)

p = GPIO.PWM(21,50)

p.start(7.5)

try:

    while True:

        p.ChangeDutyCycle(4.5)

        time.sleep(0.5)

        p.ChangeDutyCycle(10.5)

        time.sleep(0.5)

        p.ChangeDutyCycle(7.5)

        time.sleep(0.5)

except KeyboardInterrupt:

    p.stop()

    GPIO.cleanup()