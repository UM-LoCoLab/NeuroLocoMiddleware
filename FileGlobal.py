

class FileGlobalInt():
    def __init__(self, name, default=0):
        self.name=name
        try:
            fil = open(name, 'r')
        except FileNotFoundError:
            fil = open(name, 'x')
            with fil:
                fil.write("%d"%default)
            fil = open(name, 'r')
        with fil:
            self._datum = int(fil.read())

    @property
    def datum(self):
        return self._datum

    @datum.setter
    def datum(self, value):
        self._datum=value
        with open(self.name, 'w') as fil:
            fil.write("%d"%self._datum)



def test_integer():
    x = FileGlobalInt("test_file_global_integer.txt")
    print(x.datum)
    x.datum+=1
    print("%d"%x.datum)
    x.datum=0
    print("%d"%x.datum)

if __name__ == '__main__':
    test_integer()
