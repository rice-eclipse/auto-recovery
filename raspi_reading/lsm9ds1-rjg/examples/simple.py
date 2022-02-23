import sys, os
import time
from communicator import Communicator
from write_file import DataSaver

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lsm9ds1_rjg import Driver, I2CTransport, SPITransport


class SimpleExample:
    """This example shows how to poll the sensor for new data.
    It queries the sensor to discover when the accelerometer/gyro
    has new data and then reads all the sensors."""
    def __init__(self):
        # self.driver = self._create_spi_driver()
        self.driver = self._create_i2c_driver()
        self.driver.configure()
        #self.comm = Communicator('169.254.6.42', 65432)
        self.writer = DataSaver("imu_data.txt")

    @staticmethod
    def _create_i2c_driver() -> Driver:
        return Driver(
            I2CTransport(1, I2CTransport.I2C_AG_ADDRESS),
            I2CTransport(1, I2CTransport.I2C_MAG_ADDRESS))

    @staticmethod
    def _create_spi_driver() -> Driver:
        return Driver(
            SPITransport(0, False),
            SPITransport(1, True))

    def main(self):
        try:
            count = 0
            lt = 0
            while count < 50:
                lt += 1
                ag_data_ready = self.driver.read_ag_status().accelerometer_data_available
                if ag_data_ready:
                    temp,acc,gyro = self.read_ag()
                    mag = self.read_magnetometer()
                    #self.comm.send_text(f"temp: {temp}, acc: {acc}, gyro: {gyro}, mag: {mag}")
                    self.writer.save_data(acc, gyro, mag)
                    count += 1
                    print(lt)
                    lt = 0
                else:
                    time.sleep(0.00001)
        finally:
            self.driver.close()
            self.writer.close()

    def read_ag(self):
        return self.driver.read_ag_data()

    def read_magnetometer(self):
        return self.driver.read_magnetometer()


if __name__ == '__main__':
    SimpleExample().main()

