from datetime import datetime, timedelta
class DataSaver:
    def __init__(self, filename):
        self.filename = filename
        self.filenum = 1
        self.openfile()
        self.now = datetime.now()

    def openfile(self):
        self.fd = open(self.filename+str(self.wnum), "w")
        self.filenum+=1
    
    def save_data(self, a, g, m):
        if datetime.now() >= self.now + timedelta(seconds = 1):
            self.now = datetime.now()
            self.close()
            self.openfile()
        self.fd.write(f"Acc:{a} Gyr:{g} Mag:{m}\n")


    def close(self):
        self.fd.close()
		