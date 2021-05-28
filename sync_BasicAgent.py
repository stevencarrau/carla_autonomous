import carla
import queue
import numpy as np

from agents.navigation.basic_agent import BasicAgent


snapshots = queue.Queue()
image_queue = queue.Queue()
driver_queue = queue.Queue()
driver2_queue = queue.Queue()
ped_queue = queue.Queue()

states = {0:(28.8,93.4,0.0),1:(-47.1,89.9,0.0),2:(-120.5,89.6,0.0), 3:(-187.2,89.8,0.0), 4:(-190.0,4.6,0.0), 5:(-120.0,1.1,0.0), 6:(-52.2,0.8,0.0), 7:(25.5,0.5,0.0), 8:(31.1,-86.4,0.0), 9:(-48.0,-89.6,0.0), 10:(-124.0,-89.6,0.0), 11:(-188.0,-89.6,0.0)}
action_order = ['N','S','E','W']
transitions = {0:[0,7,1,0],1:[1,6,2,0],2:[2,5,3,1],3:[3,4,3,2],4:[3,11,4,5],5:[2,10,4,6],6:[1,9,5,7],7:[0,8,6,7],8:[7,8,9,8],9:[6,9,10,8],10:[5,10,11,9],11:[4,11,11,10]}

wp_path = [0,1,3,2,1,0]
init_state = 5

def carla_location(state):
    return carla.Transform(carla.Location(x=state[0], y=state[1], z=state[2]),carla.Rotation(pitch=0.000000, yaw=0.086304, roll=0.000000))


def push_snapshot(snapshot):
    snapshots.put(snapshot)


def write_res(results):
    with open("results.txt", "w") as f:
        for t, res in results:
            f.write("{:20}\t{:20}\t{:20}\t{:20}\n".format(t, res[0], res[1], res[2]))


def compare_res():
    with open("results.txt", "r") as f:
        with open("reference.txt", "r") as r:
            print("Results {}".format(f.read() == r.read()))


def get_position(frame_id, vehicle_id):
    snapshot = snapshots.get(timeout=2)
    assert(frame_id == snapshot.frame)
    transform = snapshot.find(vehicle_id).get_transform()
    return transform.location.x, transform.location.y, transform.location.z


def main():
    sim_time = 0.0
    sim_time_end = 200
    delta_time = 1.0/60.0
    results = []
    index = 0
    actor_list = []
    camera_list = []


    client = carla.Client("127.0.0.1", 2000)
    world = client.get_world()

    # set mode to synchronous
    frame_id = world.apply_settings(carla.WorldSettings(True, False, delta_time))
    print("Synchronous mode will be applied in frame: {}".format(frame_id))

    # spawn car
    spectator = world.get_spectator()
    blueprint_library = world.get_blueprint_library()
    map_ = world.get_map()
    bp = blueprint_library.filter('vehicle')[0]
    bp.set_attribute('color', '255,255,255')
    vehicle = world.spawn_actor(bp,carla_location(states[init_state]))


    ## Cameras
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    # camera_bp.set_attribute('shutter_speed', '125')
    camera_bp.set_attribute('image_size_x', '1280')
    camera_bp.set_attribute('image_size_y', '720')
    camera_transform = carla.Transform(carla.Location(x=-7.5, y=-5.5, z=5.4), carla.Rotation(pitch=-10, roll=0, yaw=25))




    camera_bp2 = blueprint_library.find('sensor.camera.rgb')
    # camera_bp2.set_attribute('shutter_speed', '125')
    camera_bp2.set_attribute('image_size_x', '1280')
    camera_bp2.set_attribute('image_size_y', '720')
    # camera_bp2.set_attribute('sensor_tick', '0.033')
    camera_bp2.set_attribute('fov', '120')
    camera_transform2 = carla.Transform(carla.Location(x=0.5,y=0, z=1.5),carla.Rotation(pitch=0,roll=0,yaw=0))





    camera_bp3 = blueprint_library.find('sensor.camera.rgb')
    camera_bp3.set_attribute('image_size_x', '1280')
    camera_bp3.set_attribute('image_size_y', '720')
    camera_transform3 = carla.Transform(carla.Location(x=-1.0, y=1.5, z=1.4), carla.Rotation(pitch=-2, roll=0, yaw=-75))
    camera_bp3.set_attribute('fov', '120')





    # camera_list.append([camera,camera2front,camera2back,camera3])



    world.tick()    # just so that I see it in the simulator
    world.on_tick(push_snapshot)

    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    camera.listen(image_queue.put)
    # camera3 = world.spawn_actor(camera_bp3, camera_transform3, attach_to=pedestrian)
    # camera3.listen(ped_queue.put)


    # camera2back = world.spawn_actor(camera_bp2, camera_transform2, attach_to=vehicle)
    # camera2front.listen(driver_queue.put)
    # camera2back.listen(driver2_queue.put)

    agent1 = BasicAgent(vehicle,60)
    agent_list = [agent1]
    actor_list.append(vehicle)
    next_wp = wp_path.pop(0)
    current_state = init_state
    location = states[transitions[current_state][next_wp]]
    agent1.set_destination(location)

    while sim_time <= sim_time_end:
        # vehicle_pos2 = vehicle2.get_location()
        if agent1.done():
            current_state = transitions[current_state][next_wp]
            next_wp = wp_path.pop(0)
            location = states[transitions[current_state][next_wp]]
            agent1.set_destination(location)
        control = agent1.run_step()
        vehicle.apply_control(control)

        frame_id = world.tick()
        sim_time += 0.05
        results.append((sim_time, get_position(frame_id, vehicle.id)))
        index += 1

    world.apply_settings(carla.WorldSettings(False, False, 0))
    client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
    camera.destroy()
    # camera3.destroy()
    # camera2front.destroy()
    # camera2back.destroy()

    write_res(results)

    for frame_no,q_i in enumerate(list(image_queue.queue)):
        if frame_no%10 == 0:
            print('Frame:%d'%frame_no)
        q_i.save_to_disk('_out/overview/%06d.png' % frame_no)

    for frame_no,q_i in enumerate(list(ped_queue.queue)):
        if frame_no%10 == 0:
            print('Frame:%d'%frame_no)
        q_i.save_to_disk('_out/pedview/%06d.png' % frame_no)

    for frame_no,q_i in enumerate(list(driver_queue.queue)):
        if frame_no%10 == 0:
            print('Frame:%d'%frame_no)
        q_i.save_to_disk('_out/driverview/%06d.png' % frame_no)

    for frame_no,q_i in enumerate(list(driver2_queue.queue)):
        if frame_no%10 == 0:
            print('Frame:%d'%frame_no)
        q_i.save_to_disk('_out/driverviewback/%06d.png' % frame_no)





if __name__ == '__main__':
    main()