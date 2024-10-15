---
layout: page
title: Dice Sorting
description: >
  Dice Sorting using ABB Robot arm and Robot Studio
hide_description: true
date: 10 April 2024
---

This project used an ABB robot arm and Cognex camera to track, pickup and sort dice. On this page I will document and breakdown key elements in our Rapid code.

You can download a .zip of our rapid files [HERE](/assets/RapidCode.zip)

## Defining Conveyor Workobject

For conveyor tracking the conveyor Workobject must be properly defined. We first created a normal work object for the static conveyor as normal with its origin defined at the vertex of the black L in the camera vision field. Ours looked like this but the exact coordinates will vary.

~~~ RAPID
PERS wobjdata conveyor:=[FALSE,TRUE,"",[[576.19,-234.78,305.27],[0.704296,0.00893523,0.0169069,0.709649]],[[0,0,0],[1,0,0,0]]]; ! Static Conveyor
~~~~

The sakai documentation talks about how you need to setup a new conveyor workobject using a 4 point method. This is incorrect. Once you have configured your counts/meter (8720 worked for us) and activate the conveyor you can just copy the coordinate data and axis configuration of the static conveyor into a new workobject. Except now we set it as a TASK PERS wobj and the two initial arguments will both be False. Like this:

~~~ RAPID
TASK PERS wobjdata wobjCNV1:=[FALSE,FALSE,"CNV1",[[576.19,-234.78,305.27],[0.704296,0.00893523,0.0169069,0.709649]],[[0,0,0],[1,0,0,0]]]; ! Moving Conveyor
~~~~

In this example "conveyor" is out static conveyor workobject and CNV1 is our dynamic workobject

Now to use CNV1 we need to do a couple of things in our main program:

First we Activate CNV1 and set ConfL to Off 

~~~ Rapid
ActUnit CNV1;
ConfL\Off;
~~~

After this we can do things like call camera data, or anything else associated with the conveyor

Afterwards we need to drop the work object and quickly pulse relevant signals and then we can attach data to our workobject

~~~RAPID
!release all objects from queue
DropWobj wobjCNV1;
PulseDO\PLength:=0.5,c1RemAllPObj;

!attaching thing to the moving conveyor
PulseDO\PLength:=0.5,c1SoftSyncSig;
WaitWobj wobjCNV1;
~~~

All together our code looks like this in the main module:

~~~Rapid
! This code will activate the conveyor and pulse the relevant signals to setup conveyor 

!using CNV1
ActUnit CNV1;
ConfL\Off;

! Get cam data
edice:=halloMain();
dice_data.dice_number:=edice.number;

!release all objects from queue
DropWobj wobjCNV1;
PulseDO\PLength:=0.5,c1RemAllPObj;

!attaching thing to the moving conveyor
PulseDO\PLength:=0.5,c1SoftSyncSig;
WaitWobj wobjCNV1;
~~~

## Image processing

All of our image processing is down internally in the Cognex Camera. I am not going to delve into that here but what is important to understand is that the Cognex camera will pass a string of data into our Rapid code. We can then parse that code to extract the x, y, rotation, and value of the dice. In order to do this we first need to initialize the camera. Luckily insight provides the following module to do so. You can read through it if you are a nerd but you mostly just need to pass the name of the vision program into this function.

~~~
    PROC InitCam()
        ! Initializing the camera persistant data / Initialisation des donn�es persistantes de la cam�ra
        CX_SetupCamera nActiveCam,strIP_Camera1,23,"admin","",-600,600,-600,600,-180,180,50\Timeout:=1;
        ! Initializing communication / Initialisation de la communication
        IF NOT CX_InitComm(nActiveCam,nErrStatus) THEN
            CX_ShowErrStatus nErrStatus;
            RETURN ;
        ENDIF
        !
        ! Checking the "Online" mode status / V�rification de l'�tat du mode "Online"
        IF CX_GetOnline(nActiveCam,nErrStatus) THEN
            TPErase;
            TPWrite "The vision is already on line";
        ELSE
            ! Requesting the "Online" mode / Demande de passage en mode "Online"
            IF CX_SetOnline(nActiveCam,\On,nErrStatus) THEN
                TPWrite "The vision has been set to on line mode";
            ELSE
                CX_ShowErrStatus nErrStatus;
                RETURN ;
            ENDIF
        ENDIF
        !
        ! Reading the vision program name / Lecture du nom du programme vision
        IF CX_GetFile(nActiveCam,strFileName,nErrStatus) THEN
            TPErase;
            TPWrite "Name of vision program: "+strFileName;
        ELSE
            ! Case execution memory of camera is empty / Cas m�moire ex�cution cam�ra vide
            ! Loading demo program from flash memory / Chargement programme de d�mo de la m�moire flash
            IF strFileName=strNULL THEN
                ! Setting camera offline / Passage cam�ra hors ligne
                bStatus:=CX_SetOnline(nActiveCam,\Off,nErrStatus);
                ! Loading "job" file / Chargement fichier "job"
                IF NOT CX_LoadFile(nActiveCam,strJobVision,nErrStatus) THEN
                    CX_ShowErrStatus nErrStatus;
                    RETURN ;
                ENDIF
                ! Setting camera on line / Passage cam�ra en ligne
                bStatus:=CX_SetOnline(nActiveCam,\On,nErrStatus);
            ELSE
                CX_ShowErrStatus nErrStatus;
                RETURN ;
            ENDIF
        ENDIF
    ENDPROC
~~~

This function is called in our custom camera module called hallomain (Thanks Paul). This module will call the camera initilziation, read the camera data and then call upon a data parsing function (Thanks again Paul) and pass the relevant data into a record called odice

~~~
FUNC Dice halloMain()
    VAR Dice o_dice; ! Declare dice variable
    InitCam; ! Init camera 
    WHILE TRUE DO ! Loop all this cool code
        IF CX_GetValue(nActiveCam,"U",87,strValue,nErrStatus) THEN ! If string data is present in U87 cell (This cell depends on camera, you will need to change this probably)
            o_dice := get_dicedata_from_string(strValue); ! Get dice data and parse
            IF NOT o_dice.pos_x = 0 THEN ! If no dice is detected the hallomain will set position to 0, we check for this case
                RETURN o_dice; ! if position=0 we return dice (Yeah this is a bit hacky but if the dice position is actually 0 we have bigger problems)
            ENDIF
        ELSE
            TPWrite "NOK",\Num:=nErrStatus; ! This will catch any weird edge cases and write NOK to console for debugging
        ENDIF
        WaitTime 0.1; ! Add brief wait so we are not overwhelming console
    ENDWHILE
    
ENDFUNC
~~~

## Tool Rotation 

When we pickup the dice we need to match the rotation of the tool to the dice. We do this using a custom function that relies heavily on the builtin EulerZYX and OrientZYX functions. We input a rob target and a desired rotation angle and will output a robtarget rotated around the base frame z axis. Our function looks like this: 

~~~
FUNC robtarget rotate_euler_absolute(robtarget old_target,robtarget rotated_target,num angle)
    VAR num anglex;
    VAR num angley;
    VAR num anglez;
    VAR orient angles;

    anglex:=EulerZYX(\X,old_target.rot);
    angley:=EulerZYX(\Y,old_target.rot);
    anglez:=EulerZYX(\Z,old_target.rot);


    anglez:=angle;
    rotated_target.rot:=OrientZYX(anglez,angley,anglex);
    RETURN rotated_target;
ENDFUNC
~~~

We can now use this rotated robtarget to pickup our dice

## Dice Pickup 

Dice pickup is fairly straight forward once the conveyor is working, we have our dice data and we have a rotated rob target. Our pickup program will move to the conveyor home position (0,0,0) offset by the dice x and y position and a predefined hover height. It will then follow the dice for a certain amount of time, lower down, close gripper, and move to the dice tray. 

First we define our target offset by the dice x and y and an additional hardcoded offset (We had to tweak this value manually, it was a bit hacky but it worked) We also need to adjust for a slight offset in the y due to the optical distortion (The farther the dice is away from the camera the bigger this offset becomes but only in y because the x position remains fairly consistent) 

This results defining the pPickup robtarget as follows:

~~~ Rapid
x:=edice.pos_x+14;
y:=edice.pos_y+11;
y:=y+k*y+d; ! This is fixing optical skew from the camera our k was 0.1 and d was -6.5
pPickup:=Offs(pPickup_rotated,x,y,0); ! Define pickup location as rotated position offset by x and y (Dice pos)
~~~

Now we can pass this robtarget into out pickup function shown here:

~~~
PROC pick_up_dice(robtarget pPickupTarget)
    
    ! Define follow time variables
    VAR stoppointdata my_followtime_1:=[3,TRUE,[0,0,0,0],0,1,"",0,0];
    VAR stoppointdata my_followtime_05:=[3,TRUE,[0,0,0,0],0,.5,"",0,0];
    VAR stoppointdata my_followtime_025:=[3,TRUE,[0,0,0,0],0,.25,"",0,0];
    VAR stoppointdata my_followtime_01:=[3,TRUE,[0,0,0,0],0,0.1,"",0,0];

    !moving to position above the conveyor whilst tracking it
    MoveL Offs(pPickupTarget,0,0,30),v300,z10,gripper\wobj:=wobjCNV1;

    !moving down with the conveyor and also closing the gripper
    MoveL pPickupTarget,v150,z0,gripper\wobj:=wobjCNV1;
    ! go to target with zone 0
    MoveL pPickupTarget,v300,z10\Inpos:=my_followtime_025,gripper\wobj:=wobjCNV1;
    WaitTime .1;
    close_gripper;

    !moving up whilst following and removing the first object from the queue, as we picked it up
    !MoveL Offs(pPickupTarget,0,0,50),v300,z10\Inpos:=my_followtime_01,gripper\wobj:=wobjCNV1;
    PulseDO\PLength:=0.5,c1Rem1PObj;
    


    !leaving the conveyor - not tracking anymore
    MoveL leaveConv,v500,z100,gripper\WObj:=conveyor;

    !dropping the conveyor wobj
    WaitTime .2;
    DropWobj wobjCNV1;
    DeactUnit CNV1;
    WaitTime .2;

ENDPROC
~~~

Please note that to follow the dice we first have to define a follow time variable like this:

~~~
VAR stoppointdata my_followtime_025:=[3,TRUE,[0,0,0,0],0,.25,"",0,0];
~~~

And now we can follow the dice like this (Note the use of \Inpos)

~~~
MoveL pPickupTarget,v300,z10\Inpos:=my_followtime_025,gripper\wobj:=wobjCNV1;
~~~

## Dice Tray logic

Once the dice is picked up we have to determine what to do with it. There are several things we have to consider:
* What is the value of the dice?
* Has that value been rolled?
* Where should the dice be placed?
  * What column (1-6)
  * What row (0 or 1)
* Once the dice is placed where should we pickup from?

Wow thats a lot of information we have to track. Luckily I wrote a cool function to help called dice_it_up.

Honestly dice_it up isn't doing anything crazy. Its mostly tracking whether there is a dice in each position through our dice_n_goal variables. If a goal is set to true a dice is in the position of the current dice value and we need to roll again. 

~~~
FUNC dice_tray dice_it_up(dice_tray dice_data)
    IF dice_data.dice_number=1 AND dice_data.goal_1=FALSE THEN
        dice_data.goal_1:=TRUE;
    ELSEIF dice_data.dice_number=2 AND dice_data.goal_2=FALSE THEN
        dice_data.goal_2:=TRUE;
    ELSEIF dice_data.dice_number=3 AND dice_data.goal_3=FALSE THEN
        dice_data.goal_3:=TRUE;
    ELSEIF dice_data.dice_number=4 AND dice_data.goal_4=FALSE THEN
        dice_data.goal_4:=TRUE;
    ELSEIF dice_data.dice_number=5 AND dice_data.goal_5=FALSE THEN
        dice_data.goal_5:=TRUE;
    ELSEIF dice_data.dice_number=6 AND dice_data.goal_6=FALSE THEN
        dice_data.goal_6:=TRUE;
    ELSE
        dice_data.roll_again:=TRUE;
        RETURN dice_data;
    ENDIF

    dice_data.roll_again:=FALSE;
    RETURN dice_data;

ENDFUNC
~~~

The rest of the logic is done in the main module shown below. We have a dice pickup and drop off func and all of the dice logic variables are stored in our dice_data record. 

~~~
IF dice_data.roll_again THEN ! If dropoff place is already occupied roll again
    DropoffDiceViaHome;

! Check dice tray state and determine dropoff and pickup location
ELSEIF dice_data.tray_state=FALSE THEN
    !dropoff place still empty
    dice_data.row_number:=1;
    TRAY_Dropoff(dice_data);
    dice_data.row_number:=0;
    dice_data.pickup_location:=dice_data.pickup_location+1;
    IF dice_data.pickup_location>6 THEN
        dice_data.pickup_location:=1;
        dice_data.tray_state:=TRUE;
        dice_data.row_number := 1;
        dice_data := reset_dice(dice_data);
    ENDIF
    TRAY_Pickup(dice_data);
    DropoffDiceViaHome;
ELSEIF dice_data.tray_state=TRUE THEN
    !dropoff place still empty
    dice_data.row_number:=0;
    TRAY_Dropoff(dice_data);
    dice_data.row_number:=1;
    dice_data.pickup_location:=dice_data.pickup_location+1;
    IF dice_data.pickup_location>6 THEN
        dice_data.pickup_location:=1;
        dice_data.tray_state:=FALSE;
        dice_data.row_number := 0;
        dice_data := reset_dice(dice_data);
    ENDIF
    TRAY_Pickup(dice_data);
    DropoffDiceViaHome;
ENDIF
~~~

