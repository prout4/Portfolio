# ME371 - Gearbox

This project required groups to design and manufacture a gearbox to raise, lower, and hold a 5kg mass at fixed speeds. 
The gearbox recieves a fixed 60 rpm input and shifts between 2 drive gears and a neutral position. The gearbox must automatically brake when in the neutral position.

Note that the entire design is 3D printed (except for worm gear in V0.5 and drive shafts).

## V0.5
This design uses a worm gear to brake when in the neutral position since a worm gear with sufficiently high gear ratio cannot be backdriven. To meet speed requirements, the input to the worm gear was required to rotate at approximately 960 rpm. This required a large gearbox with significant loss through the geartrain. This design ultimately failed since the belt used to provide the 60 rpm input slipped (and could not be sufficiently tensioned).

![v0.5 iso view](/Mechanical%20Course%20Projects/Year%203%20-%20Gearbox/Isometric.png)

## V1.0
This is a complete redesign of the gearbox, keeping only a similar shifter mechanism from V0.5.

Braking is now accomplished using a moving [hardstop](#shifterBrakeBlock). This reduces the overall load on the gearset since the hardstop takes all of the load from braking (compared to the previous design where a worm gear was taking the load).

![v1.0 iso view](/Mechanical%20Course%20Projects/Year%203%20-%20Gearbox/NewIso.png)

<a name="shifterBrakeBlock"></a>
![v1.0 shifter](/Mechanical%20Course%20Projects/Year%203%20-%20Gearbox/NewSection.png) 

![v1.0 shifter](/Mechanical%20Course%20Projects/Year%203%20-%20Gearbox/NewSectionIso.png)