#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random
import time


def main():
    actor_list = []

    # In this tutorial script, we are going to add a vehicle to the simulation
    # and let it drive in autopilot. We will also create a camera attached to
    # that vehicle, and save all the images generated by the camera to disk.

    try:
        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)


        # client.start_recorder('/home/gradandpostdoc/Videos/CarlaRecordings/Logs/record.log')


        # Once we have a client we can retrieve the world that is currently
        # running.
        world = client.get_world()
        # settings = world.get_settings()
        # settings.synchronous_mode = True
        # settings.fixed_delta_seconds = 0.05
        # world.apply_settings(settings)

        # The world contains the list blueprints that we can use for adding new
        # actors into the simulation.
        blueprint_library = world.get_blueprint_library()

        # Now let's filter all the blueprints of type 'vehicle' and choose one
        # at random.
        # bp = random.choice(blueprint_library.filter('vehicle'))
        bp = blueprint_library.filter('vehicle')[0]

        # A blueprint contains the list of attributes that define a vehicle's
        # instance, we can read them and modify some of them. For instance,
        # let's randomize its color.
        # if bp.has_attribute('color'):
        #     color = random.choice(bp.get_attribute('color').recommended_values)
        #     bp.set_attribute('color', color)

        # Now we need to give an initial transform to the vehicle. We choose a
        # random transform from the list of recommended spawn points of the map.
        spawn_points = world.get_map().get_spawn_points()
        # spawn_points = [spawn_points[24],spawn_points[156]]
        transform = spawn_points[0]


        # So let's tell the world to spawn the vehicle.
        bp.set_attribute('color', '255,255,255')
        vehicle = world.spawn_actor(bp, transform)
        # if bp.has_attribute('color'):
        #     color = random.choice(bp.get_attribute('color').recommended_values)
        #     bp.set_attribute('color', color)
        bp.set_attribute('color', '224,0,0')
        vehicle2 = world.spawn_actor(bp,spawn_points[1])

        safe_spawn = spawn_points[2]
        safe_spawn.location.y= 67.45
        safe_spawn.rotation.yaw= 285

        # pedestrian = world.spawn_actor(blueprint_library[90],spawn_points[2])
        pedestrian = world.spawn_actor(blueprint_library[90],safe_spawn)

        # It is important to note that the actors we create won't be destroyed
        # unless we call their "destroy" function. If we fail to call "destroy"
        # they will stay in the simulation even after we quit the Python script.
        # For that reason, we are storing all the actors we create so we can
        # destroy them afterwards.
        actor_list.append(vehicle)
        actor_list.append(vehicle2)
        actor_list.append(pedestrian)

        ## Camera 1
        # camera_bp = blueprint_library.find('sensor.camera.rgb')
        # # camera_bp.set_attribute('shutter_speed', '125')
        # camera_bp.set_attribute('image_size_x', '1280')
        # camera_bp.set_attribute('image_size_y', '720')
        # camera_transform = carla.Transform(carla.Location(x=-7.5,y=-5.5, z=5.4),carla.Rotation(pitch=-10,roll=0,yaw=25))
        # camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        # actor_list.append(camera)
        # # cc = carla.ColorConverter.CityScapesPalette
        # # camera.listen(lambda image: image.save_to_disk('_out/overview/%06d.png' % image.frame))
        # print('created %s' % camera.type_id)

        ## Camera 2
        # camera_bp2 = blueprint_library.find('sensor.camera.rgb')
        # camera_bp2.set_attribute('shutter_speed', '125')
        # camera_bp2.set_attribute('image_size_x', '320')
        # camera_bp2.set_attribute('image_size_y', '180')
        # # camera_bp2.set_attribute('sensor_tick', '0.033')
        # camera_bp2.set_attribute('fov', '120')
        # camera_transform2 = carla.Transform(carla.Location(x=0.5,y=0, z=1.5),carla.Rotation(pitch=0,roll=0,yaw=0))
        # camera2 = world.spawn_actor(camera_bp2, camera_transform2, attach_to=vehicle)
        # actor_list.append(camera2)
        # camera2.listen(lambda image: image.save_to_disk('_out/driverview/%06d.png' % image.frame))
        # print('created %s' % camera2.type_id)

        # carla_settings.add_sensor(camera)

        camera_bp3 = blueprint_library.find('sensor.camera.rgb')
        camera_bp3.set_attribute('image_size_x', '320')
        camera_bp3.set_attribute('image_size_y', '180')
        camera_transform3 = carla.Transform(carla.Location(x=-1.0, y=1.5, z=1.4),carla.Rotation(pitch=-2, roll=0, yaw=-75))
        camera_bp3.set_attribute('fov', '120')
        camera3 = world.spawn_actor(camera_bp3, camera_transform3, attach_to=pedestrian)
        camera3.listen(lambda image: image.save_to_disk('_out/pedview/%06d.png' % image.frame))

        # time.sleep(5)
        start_t = time.time()
        vehicle.apply_control(carla.VehicleControl(throttle=0.45))
        vehicle2.apply_control(carla.VehicleControl(throttle=0.45))
        vehicle_pos2 = vehicle2.get_location()

        # Front Stop Control Loop
        while vehicle_pos2.x< 145:
            vehicle_pos = vehicle.get_location()
            vehicle_pos2 = vehicle2.get_location()
            if vehicle_pos2.x>130:
                control = carla.WalkerControl()
                control.speed = 0.95
                control.direction.y = -1
                control.direction.x = 0.0  # 15
                pedestrian.apply_control(control)


        vehicle2.apply_control(carla.VehicleControl(throttle=0.0,brake=0.8))
        # if time.time()-start_t>30.0:
        #     break

        # # Safe Control Loop
        # while vehicle_pos2.x < 175:
        #     vehicle_pos = vehicle.get_location()
        #     vehicle_pos2 = vehicle2.get_location()
        #
        # vehicle2.apply_control(carla.VehicleControl(throttle=0.0,brake=0.8))
        # vehicle.apply_control(carla.VehicleControl(throttle=0.0,brake=0.8))

        # # # Front Stop Back Stop Control Loop
        # while vehicle_pos2.x< 145:
        #     vehicle_pos = vehicle.get_location()
        #     vehicle_pos2 = vehicle2.get_location()
        #
        # vehicle.apply_control(carla.VehicleControl(throttle=0.0,brake=0.8))
        # vehicle2.apply_control(carla.VehicleControl(throttle=0.0, brake=0.8))




        # # Front Pass Back Stop Control Loop
        # while vehicle_pos2.x< 145:
        #     vehicle_pos = vehicle.get_location()
        #     vehicle_pos2 = vehicle2.get_location()
        #
        # vehicle.apply_control(carla.VehicleControl(throttle=0.0,brake=0.8))
        #
        # while vehicle_pos2.x < 175:
        #     vehicle_pos2 = vehicle2.get_location()
        # vehicle2.apply_control(carla.VehicleControl(throttle=0.0, brake=0.8))

        # print('created %s' % vehicle.type_id)
        # actor_list.append(vehicle2)
        # print('created %s' % vehicle2.type_id)

        # for i in range()


        # Let's put the vehicle to drive around.
        # vehicle.set_autopilot(True)

        # Let's add now a "depth" camera attached to the vehicle. Note that the
        # transform we give here is now relative to the vehicle.
        # camera_bp = blueprint_library.find('sensor.camera.depth')
        # camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        # camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        # actor_list.append(camera)
        # print('created %s' % camera.type_id)

        # Now we register the function that will be called each time the sensor
        # receives an image. In this example we are saving the image to disk
        # converting the pixels to gray-scale.
        # cc = carla.ColorConverter.LogarithmicDepth
        # camera.listen(lambda image: image.save_to_disk('_out/%06d.png' % image.frame, cc))

        # Oh wait, I don't like the location we gave to the vehicle, I'm going
        # to move it a bit forward.
        # location = vehicle.get_location()
        # location.x += 0
        # vehicle.set_location(location)
        # print('moved vehicle to %s' % location)

        # But the city now is probably quite empty, let's add a few more
        # vehicles.
        # transform.location += carla.Location(x=40, y=-3.2)
        # transform.rotation.yaw = -180.0
        # for _ in range(0, 10):
        #     transform.location.x += 8.0
        #
        #     bp = random.choice(blueprint_library.filter('vehicle'))
        #
        #     # This time we are using try_spawn_actor. If the spot is already
        #     # occupied by another object, the function will return None.
        #     npc = world.try_spawn_actor(bp, transform)
        #     if npc is not None:
        #         actor_list.append(npc)
        #         npc.set_autopilot()
        #         print('created %s' % npc.type_id)

        time.sleep(5)
        # client.stop_recorder()

    finally:

        # print('Runtime: {}'.format(time.time()-start_t))
        print('destroying actors')
        # camera.destroy()
        # camera2.destroy()
        camera3.destroy()
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('done.')


if __name__ == '__main__':

    main()