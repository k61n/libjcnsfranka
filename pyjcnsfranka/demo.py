
from pyjcnsfranka.routines import cryo


if __name__ == '__main__':
    for i in range(100):
        cryo.load()
        cryo.unload()
        print(f'Iteration {i} success')
