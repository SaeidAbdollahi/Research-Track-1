Python Robotics Simulator
================================

This is a simple, portable robot simulator developed by [Student Robotics](https://studentrobotics.org).
Some of the arenas and the exercises have been modified for the Research Track I course

Installing and running
----------------------

The simulator requires a Python 2.7 installation, the [pygame](http://pygame.org/) library, [PyPyBox2D](https://pypi.python.org/pypi/pypybox2d/2.1-r331), and [PyYAML](https://pypi.python.org/pypi/PyYAML/).


You can run the program with:

```bash
$ python2.7 run.py
```

and after that enter the name of the .py file 'SaeedAbdollahiTaromsary.py' as below:

```bash
pygame 2.0.3 (SDL 2.0.16, Python 2.7.18)
Hello from the pygame community. https://www.pygame.org/contribute.html
Using C extension
Enter the names of the Python files to run, separated by commas: SaeedAbdollahiTaromsary.py
```

![alt text](https://github.com/SaeidAbdollahi/Research-Track-1/blob/main/Assignment%201/Simulation%20Pictures/start.png?raw=true)

## Troubleshooting

When running `python run.py <file>`, you may be presented with an error: `ImportError: No module named 'robot'`. This may be due to a conflict between sr.tools and sr.robot. To resolve, symlink simulator/sr/robot to the location of sr.tools.

On Ubuntu, this can be accomplished by:
* Find the location of srtools: `pip show sr.tools`
* Get the location. In my case this was `/usr/local/lib/python2.7/dist-packages`
* Create symlink: `ln -s path/to/simulator/sr/robot /usr/local/lib/python2.7/dist-packages/sr/`

Robot API
---------

The API for controlling a simulated robot is designed to be as similar as possible to the [SR API][sr-api].

### Motors ###

The simulated robot has two motors configured for skid steering, connected to a two-output [Motor Board](https://studentrobotics.org/docs/kit/motor_board). The left motor is connected to output `0` and the right motor to output `1`.

The Motor Board API is identical to [that of the SR API](https://studentrobotics.org/docs/programming/sr/motors/), except that motor boards cannot be addressed by serial number. So, to turn on the spot at one quarter of full power, one might write the following:

```python
R.motors[0].m0.power = 25
R.motors[0].m1.power = -25
```

### The Grabber ###

The robot is equipped with a grabber, capable of picking up a token which is in front of the robot and within 0.4 metres of the robot's centre. To pick up a token, call the `R.grab` method:

```python
success = R.grab()
```

The `R.grab` function returns `True` if a token was successfully picked up, or `False` otherwise. If the robot is already holding a token, it will throw an `AlreadyHoldingSomethingException`.

To drop the token, call the `R.release` method.

Cable-tie flails are not implemented.

### Vision ###

To help the robot find tokens and navigate, each token has markers stuck to it, as does each wall. The `R.see` method returns a list of all the markers the robot can see, as `Marker` objects. The robot can only see markers which it is facing towards.

Each `Marker` object has the following attributes:

* `info`: a `MarkerInfo` object describing the marker itself. Has the following attributes:
  * `code`: the numeric code of the marker.
  * `marker_type`: the type of object the marker is attached to (either `MARKER_TOKEN_GOLD`, `MARKER_TOKEN_SILVER` or `MARKER_ARENA`).
  * `offset`: offset of the numeric code of the marker from the lowest numbered marker of its type. For example, token number 3 has the code 43, but offset 3.
  * `size`: the size that the marker would be in the real game, for compatibility with the SR API.
* `centre`: the location of the marker in polar coordinates, as a `PolarCoord` object. Has the following attributes:
  * `length`: the distance from the centre of the robot to the object (in metres).
  * `rot_y`: rotation about the Y axis in degrees.
* `dist`: an alias for `centre.length`
* `res`: the value of the `res` parameter of `R.see`, for compatibility with the SR API.
* `rot_y`: an alias for `centre.rot_y`
* `timestamp`: the time at which the marker was seen (when `R.see` was called).

For example, the following code lists all of the markers the robot can see:

```python
markers = R.see()
print "I can see", len(markers), "markers:"

for m in markers:
    if m.info.marker_type in (MARKER_TOKEN_GOLD, MARKER_TOKEN_SILVER):
        print " - Token {0} is {1} metres away".format( m.info.offset, m.dist )
    elif m.info.marker_type == MARKER_ARENA:
        print " - Arena marker {0} is {1} metres away".format( m.info.offset, m.dist )
```

[sr-api]: https://studentrobotics.org/docs/programming/sr/

## Pseudocode of the Implemented Algorithm

This section describes the pseudocode of the implemented algorithm for handling the robot's task.

```python
print("*******************Robot Task Starts*******************")
while 1:
	Try to find a silver_object
	if no silver_bject found:
		do a random movement
	if  a silver_object found:
		move robot to the silver_object
	elif robot grabbed the silver_object:
		while 1:
			now try to find a gold_object
			if no gold_object found:
				do a random movement
			move robot to the gold_object
			if the robot put the silver_object near the gold_object:
				now release the silver_object and try to do another task
				break
```

Starts the task:

Moves to the Silver Object:

![alt text](https://github.com/SaeidAbdollahi/Research-Track-1/blob/main/Assignment%201/Simulation%20Pictures/start.png?raw=true)

Grabes the Silver Object:

![alt text](https://github.com/SaeidAbdollahi/Research-Track-1/blob/main/Assignment%201/Simulation%20Pictures/pick_obj.png?raw=true)

Releases the Silver Object Near a Gold Object:

![alt text](https://github.com/SaeidAbdollahi/Research-Track-1/blob/main/Assignment%201/Simulation%20Pictures/put_obj.png?raw=true)

Doing the Last Task:

![alt text](https://github.com/SaeidAbdollahi/Research-Track-1/blob/main/Assignment%201/Simulation%20Pictures/put_the_last_obj.png?raw=true)

All Tasks Done:

![alt text](https://github.com/SaeidAbdollahi/Research-Track-1/blob/main/Assignment%201/Simulation%20Pictures/finish.png?raw=true)

## Possible Improvements

A challenge in performing each task is the movement of the robot towards the silver objects that have already been moved, which means that in the previous stages, the robot has moved the silver object and placed it near the golden object, but at the beginning of the new task, it is possible to try again to move the previous silver objects.

A solution for this problem is assigning IDs to silver objects and creating a list of completed tasks, in this case, after moving each silver object and placing it near the golden object, the ID of the silver object should be put in the list of completed tasks so that the robot does not go to that silver object again in the next turns.
