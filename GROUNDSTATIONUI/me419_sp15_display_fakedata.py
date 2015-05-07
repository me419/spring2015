import random

def datastream():
    inttemp = random.uniform(10,25)
    exttemp = random.uniform(10,25)
    airpres = random.uniform(100000,102000)
    lon = random.uniform(10,350)
    lat = random.uniform(10,350)
    alt = random.uniform(10,350)
    velocity = random.uniform(0.1,0.6)
    roll = random.uniform(10,350)
    pitch = random.uniform(10,350)
    yaw = random.uniform(10,350)
    rollrate = random.uniform(1,90)
    pitchrate = random.uniform(1,90)
    yawrate = random.uniform(1,90)
    voltage = random.uniform(4.9,5.1)

    
    stream = [inttemp,exttemp,airpres,lon,lat,alt,velocity,
                     roll,pitch,yaw,rollrate,pitchrate,yawrate,voltage]
    yield ','.join([str(i) for i in stream])
    while True:
        yield ','.join([str(i+random.uniform(-i/100.,i/100.)) for i in stream])

