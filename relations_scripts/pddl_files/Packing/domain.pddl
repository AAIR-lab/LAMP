(define (domain Packing)
(:requirements :strips :typing :equality :conditional-effects :existential-preconditions :universal-preconditions)
(:types 
	gripper
	can
	surface
)

(:predicates 
	(gripper_can_0 ?x - gripper ?y - can)
	(gripper_can_1 ?x - gripper ?y - can)
	(gripper_can_2 ?x - gripper ?y - can)
	(can_surface_0 ?x - can ?y - surface)
	(can_surface_1 ?x - can ?y - surface)
	(clear3_gripper_can_1 ?x - gripper)
	(clear3_gripper_can_2 ?x - gripper)
)

(:action a1 
:parameters ( ?can_p1 - can  ?surface_extra_p1 - surface  ?gripper_p1 - gripper )
:precondition (and 
	(can_surface_1 ?can_p1 ?surface_extra_p1)
	(gripper_can_2 ?gripper_p1 ?can_p1)
	(clear3_gripper_can_1 ?gripper_p1) 
) 
:effect (and 
 	(gripper_can_0 ?gripper_p1 ?can_p1) 
	(not (gripper_can_1 ?gripper_p1 ?can_p1))
	(not (gripper_can_2 ?gripper_p1 ?can_p1))
	(clear3_gripper_can_2 ?gripper_p1) 
 ) 
)

(:action a2 
:parameters ( ?can_p1 - can  ?gripper_p1 - gripper )
:precondition (and 
	(gripper_can_2 ?gripper_p1 ?can_p1)
	(clear3_gripper_can_1 ?gripper_p1) 
) 
:effect (and 
 	(gripper_can_1 ?gripper_p1 ?can_p1) 
	(not (gripper_can_0 ?gripper_p1 ?can_p1))
	(not (gripper_can_2 ?gripper_p1 ?can_p1))
	(clear3_gripper_can_2 ?gripper_p1) 
	(not (clear3_gripper_can_1 ?gripper_p1)) 
 ) 
)

(:action a3 
:parameters ( ?can_p1 - can  ?surface_extra_p1 - surface  ?gripper_p1 - gripper )
:precondition (and 
	(can_surface_1 ?can_p1 ?surface_extra_p1)
	(gripper_can_1 ?gripper_p1 ?can_p1)
	(clear3_gripper_can_2 ?gripper_p1) 
) 
:effect (and 
 	(gripper_can_2 ?gripper_p1 ?can_p1) 
	(not (gripper_can_0 ?gripper_p1 ?can_p1))
	(not (gripper_can_1 ?gripper_p1 ?can_p1))
	(clear3_gripper_can_1 ?gripper_p1) 
	(not (clear3_gripper_can_2 ?gripper_p1)) 
 ) 
)

(:action a4 
:parameters ( ?can_p1 - can  ?gripper_p1 - gripper )
:precondition (and 
	(gripper_can_0 ?gripper_p1 ?can_p1)
	(clear3_gripper_can_1 ?gripper_p1) 
	(clear3_gripper_can_2 ?gripper_p1) 
) 
:effect (and 
 	(gripper_can_2 ?gripper_p1 ?can_p1) 
	(not (gripper_can_0 ?gripper_p1 ?can_p1))
	(not (gripper_can_1 ?gripper_p1 ?can_p1))
	(not (clear3_gripper_can_2 ?gripper_p1)) 
 ) 
)

(:action a5 
:parameters ( ?can_p1 - can  ?gripper_extra_p1 - gripper  ?surface_p1 - surface )
:precondition (and 
	(can_surface_0 ?can_p1 ?surface_p1)
	(gripper_can_1 ?gripper_extra_p1 ?can_p1)
) 
:effect (and 
 	(can_surface_1 ?can_p1 ?surface_p1) 
	(not (can_surface_0 ?can_p1 ?surface_p1))
 ) 
)


)