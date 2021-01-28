from roku import Roku
import time

def tube():
    roku = Roku("10.0.0.66")
    roku.home()
    time.sleep(1)
    youtube = roku["YouTube"]
    youtube.launch()
    time.sleep(5)
    roku.left()
    roku.up()
    roku.right()
    roku.right()
    roku.literal("congratulations guy")
    time.sleep(1)
    roku.down()
    roku.down()
    roku.down()
    roku.down()
    roku.down()
    roku.select()
    roku.volume_up()

