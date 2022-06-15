
# Fine odometry function

The function **getFineOdometry()** allows to retrieve a more precise position distance of the SWD Core at low level speed.

> This position is non-safe. Safety relevant applications should rely with **getOdometryValue()**.

This function use the precise multiturn position value (**hall_encoder**), in units of motor mechanical degrees.

> The counter direction depends of the position polarity parameter.

## COMMISSIONING
In order to switch from **position_value** to **hall_encoder** data in the PDO mapping , the commissioning of the both motors have to be modifed as follow:
```
commissioning.py:

    # TPDO3 communication mapping

    # - statusword
    # - position_value or hall_encoder

    mapping = PDOMappingParameters()
    mapping.nb = 2
    mapping.items.append(0x6041_00_10)
    # mapping.items.append(0x6064_00_20) # position_value
    mapping.items.append(0x2305_06_20)  # hall_encoder

    communication_client.setTPDOMappingParameters(PDOId.PDO_3, mapping)
    check("setTPDOMappingParameters(PDOId.PDO_3)", error)
 ```
## ROS2
The latest swd_diff_drive controller on github contains a parameter to indicate which odometry shall be used: `"fine_odometry": False.`By default, this parameter is set to false in order to continue using `position_value`data and `getOdometryValue()` function.

## ROS1
Until **getFineOdometry()** is available, a possible workaround is to modif "**nbPolePair**" setting into `configuration.json` file :
	`"nbPolePair": 60.0` instead of `"nbPolePair": 5.0`

Thus, the formula used by "**getOdometryValue**" will compute the same value as   "**getFineOdometryValue**" and all transparent to the swd_diff_drive_controller node.
