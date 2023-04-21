
from pyjcnsfranka.routines import cryo_stick


if __name__ == '__main__':
    for i in range(100):
        cryo_stick.load()
        cryo_stick.unload()
        print(f'Iteration {i} success')
