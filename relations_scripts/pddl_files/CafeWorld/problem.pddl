(define (problem can_surface_1)
(:domain CafeWorld)(:objects 
	freight_1 - freight
	can_1 - can
	gripper_1 - gripper
	surface_5 - surface
	surface_4 - surface
	surface_1 - surface
	surface_0 - surface
	surface_3 - surface
	surface_2 - surface
)
(:init 
 	(can_surface_0 can_1 surface_0) (can_surface_0 can_1 surface_1) (can_surface_0 can_1 surface_2) (can_surface_0 can_1 surface_3) (can_surface_0 can_1 surface_5) 
(can_surface_1 can_1 surface_4) (freight_can_0 freight_1 can_1) (freight_gripper_1 freight_1 gripper_1) (freight_surface_0 freight_1 surface_0) 
(freight_surface_0 freight_1 surface_1) (freight_surface_0 freight_1 surface_2) (freight_surface_0 freight_1 surface_3) (freight_surface_0 freight_1 surface_4) 
(freight_surface_0 freight_1 surface_5) (gripper_can_0 gripper_1 can_1) (clear3_freight_can_1 freight_1) (clear3_freight_surface_1 freight_1) (clear3_gripper_can_1 gripper_1) (clear3_gripper_can_2 gripper_1)  
 )
(:goal 
 	(and (can_surface_1 can_1 surface_5) (freight_can_0 freight_1 can_1) (freight_gripper_1 freight_1 gripper_1) (freight_surface_0 freight_1 surface_0) (freight_surface_0 freight_1 surface_1) 
(freight_surface_0 freight_1 surface_2) (freight_surface_0 freight_1 surface_3) (freight_surface_0 freight_1 surface_4) (freight_surface_0 freight_1 surface_5) 
(gripper_can_0 gripper_1 can_1) ) 
 )
 )