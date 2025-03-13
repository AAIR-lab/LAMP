(define (problem can_surface_1)
(:domain Packing)(:objects 
	gripper_1 - gripper
	can_1 - can
	surface_1 - surface
)
(:init 
 	(can_surface_0 can_1 surface_1) (gripper_can_0 gripper_1 can_1) (clear3_gripper_can_1 gripper_1) (clear3_gripper_can_2 gripper_1)  
 )
(:goal 
 	(and (can_surface_1 can_1 surface_1) (gripper_can_0 gripper_1 can_1) ) 
 )
 )