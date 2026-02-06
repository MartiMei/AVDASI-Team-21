# Corncorde UAV Avionic Scripts
## Description
This is team 21's avionic script's github page, we focus on creating the two-end control with suitable UI for UAV coursework project, while achieving the requirements mentioned in next section.

## Requirements
See in: https://uob.sharepoint.com/teams/UnitTeams-CADE20005-2025-26-TB-4-A/Shared%20Documents/Forms/AllItems.aspx?id=%2Fteams%2FUnitTeams%2DCADE20005%2D2025%2D26%2DTB%2D4%2DA%2FShared%20Documents%2FGeneral%2F2025%20AVDASI2%20Requirements%20Specification%2Epdf&parent=%2Fteams%2FUnitTeams%2DCADE20005%2D2025%2D26%2DTB%2D4%2DA%2FShared%20Documents%2FGeneral

## Roadmap

### Current Version (v10.2)
- UI update


### Ideas
- Add cmd logging for sensor part(Done)
- UI improvement(Done partly)
- 3D plot improvement(Done partly, might add moveable componet animation in future)
- Delay Reduce(Done partly)
- Auto download of csv log from MP after flight by lua
- Frame addition to left, stop UI stretching everytime number changes
### Previous Versions
- v10.1: Minor changes

- v10: Live plots removed to reduce the risk of lag in the program - Live data shown instead;no lag. All plotting will be done after wind tunnel test.

-v9.8: 3D plots improvement: form 3 basic lines to a basic modeled plane shape, add axes of pitch/yaw/row. Might need to change the view to suit wind tunnel testing sequece.

-v9.7: Functionality with updated Lua code - data logging done directly to SD Card on the Cube at 20Hz

-v9.5: Minor changes

-v9.4: New functions: update to allow 2 servos to rotate simultaneously for elevator, and 2 servos to rotate simultaneously for the flap

-v9.3: UI updates: Rearrange base on new design, Safety Lock Button added (UI test purposes only), 

-v9.0:cmd logging， bug fix, redundancy

-v8.1:  bug fix

-v8.0: pitch rate, plot of PWM vs time, csv logging of flight

-v7.5:Fix bugs, csv output

-v7.4:Intergrate with lua script

-v7.2.1: Senser plot lagging solved

-v7.2:Max/min servo test, potentialmeter plot

-v6.4:Updates on flap sensing and UI

-v6.3:Direct angle input, on-time PWM display, 3D altitude view

-v5.1:UI updates, control response plots, 

-v3: Reset positions(Center), status logging plot

-v2: First python version, data logging, basic UI, basic plot of Elevator/Aileron/Rudder VS Pitch/Roll/Yaw, quit button, slides for angle change

## Contributions to the code

-MM:

-SS:



