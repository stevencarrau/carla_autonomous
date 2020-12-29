import carla
import queue
import numpy as np


snapshots = queue.Queue()
image_queue = queue.Queue()
driver_queue = queue.Queue()
driver2_queue = queue.Queue()
ped_queue = queue.Queue()



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
    sim_time_end = 45.0
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

    spawn_points = world.get_map().get_spawn_points()
    bp.set_attribute('color', '255,255,255')
    vehicle = world.spawn_actor(bp,spawn_points[0])
    bp.set_attribute('color', '224,0,0')
    vehicle2 = world.spawn_actor(bp, spawn_points[1])
    safe_spawn = spawn_points[2]
    safe_spawn.location.y = 67.45
    safe_spawn.rotation.yaw = 285
    pedestrian = world.spawn_actor(blueprint_library[90], safe_spawn)
    actor_list.append(vehicle)
    actor_list.append(vehicle2)
    actor_list.append(pedestrian)

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
    #
    # camera2front = world.spawn_actor(camera_bp2, camera_transform2, attach_to=vehicle2)
    # camera2back = world.spawn_actor(camera_bp2, camera_transform2, attach_to=vehicle)
    # camera2front.listen(driver_queue.put)
    # camera2back.listen(driver2_queue.put)

    while sim_time <= sim_time_end:
        vehicle_pos2 = vehicle2.get_location()
        if vehicle_pos2.x > 145:
            # vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.8))
            vehicle2.apply_control(carla.VehicleControl(throttle=0.0, brake=0.8))
        else:
            control = carla.WalkerControl()
            control.speed = 0.95
            control.direction.y = -1
            control.direction.x = 0.0  # 15
            pedestrian.apply_control(control)
            # vehicle.apply_control(carla.VehicleControl(throttle=0.45))  # vehicle.set_velocity(carla.Vector3D(y=velocity[index]))
            # vehicle2.apply_control(carla.VehicleControl(throttle=0.45))  # vehicle.set_velocity(carla.Vector3D(y=velocity[index]))
            # if vehicle_pos2.x>130:


        # client.apply_batch_sync([carla.command.ApplyVelocity(vehicle.id, carla.Vector3D(y=velocity[index]))])
        print("before tick")
        frame_id = world.tick()
        print("after tick {}".format(frame_id))
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