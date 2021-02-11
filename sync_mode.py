import glob
import os
import sys

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
import queue
import numpy as np

#snapshots = queue.Queue()
#image_queue = queue.Queue()


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context
        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)
    """

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data

#def push_snapshot(snapshot):
#    snapshots.put(snapshot)


#def write_res(results):
#    with open("results.txt", "w") as f:
#        for t, res in results:
#            f.write("{:20}\t{:20}\t{:20}\t{:20}\n".format(t, res[0], res[1], res[2]))

def main():
    actor_list = []

    #clock = time.time()
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()

    try:
        m = world.get_map()
        #start_pose = random.choice(m.get_spawn_points())
        spawn_point = world.get_map().get_spawn_points()
        transform = spawn_point[0]
       
        waypoint = m.get_waypoint(transform.location)

        blueprint_library = world.get_blueprint_library()
        vehicle_blueprint = client.get_world().get_blueprint_library().filter('model3')[0]

        #vehicle = world.spawn_actor(
        #    random.choice(blueprint_library.filter('vehicle.*')),
        #    start_pose)

        vehicle = client.get_world().spawn_actor(vehicle_blueprint, transform)
        actor_list.append(vehicle)
        vehicle.set_simulate_physics(False)

        camera_rgb = world.spawn_actor(
            blueprint_library.find('sensor.camera.rgb'),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            attach_to=vehicle)
        actor_list.append(camera_rgb)
        
        #driver_queue = queue.Queue()
        #camera_rgb.listen(driver_queue.put)
        

        with CarlaSyncMode(world, camera_rgb, fps=30) as sync_mode:
            while True:
                #if should_quit():
                #    return

                try:
                    world.tick()
                    
                    # Advance the simulation and wait for the data.
                    snapshot, imge_rgb = sync_mode.tick(timeout=2.0)

                    # Choose the next waypoint and update the car location.
                    waypoint = random.choice(waypoint.next(1.5))
                    vehicle.set_transform(waypoint.transform)

                except KeyboardInterrupt:
                    print('\nKilling loop!')
                    return
        
        
        #
        #    write_res(results)

        #for frame_no,q_i in enumerate(list(driver_queue.queue)):
        #    if frame_no%10 == 0:
        #        print('Frame:%d'%frame_no)
        #    q_i.save_to_disk('_out/driverview/%06d.png' % frame_no)


    finally:

        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()

        print('done.')

if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
