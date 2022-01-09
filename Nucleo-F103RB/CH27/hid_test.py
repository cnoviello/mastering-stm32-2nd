import hid, time, threading, struct

class myT(threading.Thread):
    def run(self):
        global hidhandle
        while True:
            report = hidhandle.read(0x4)
            if report[1] == 1:
                print("USER BUTTON PRESSED")
            elif report[1] == 0:
                print("USER BUTTON RELEASED")

hidhandle = hid.device()
hidhandle.open(0x483, 0x5750)

print("Manufacturer: %s" % hidhandle.get_manufacturer_string())
print("Product: %s" % hidhandle.get_product_string())
print("Serial No: %s" % hidhandle.get_serial_number_string())

t = myT()
t.start()

ledStatus = 1
while True:
    p = tuple(struct.pack(">H", ledStatus))
    hidhandle.write((2,0)+p)
    time.sleep(1)
    ledStatus = not ledStatus