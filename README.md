# Wearhouse_dist_RB_Arm_PART

In this project, a Robotic Arm picks up the items from a conveyor belt and places it on a transportation vehicle, that distributes the item or takes it to further processing. The Robotic Arm is operated by a Raspberry Pi (4), and the distribution vehicle is operated by an Arduino (Mega). This is a part of a larger Wearhouse Automation System.

Here the items that are to be distributed will be picked up by the robotic arm from the conveyor and finds the Transportation vehicle using a PiCamera with the help of OpenCV (vehicle was chosen to be marked in yellow for the project). The Raspberry Pi then places the item on the vehicle and then sends a signal confirming the placement. Bluetooth was the mode of communication for the project.

## Scope of Expansion
To develop a fully autonomous warehouse system, with AI-based identification and distribution. The transportation vehicle can be incorporated with pathfinding and Human Machine Interface mechanisms to allow human interference in the Wearhouse (for warehouses that need human factor).
