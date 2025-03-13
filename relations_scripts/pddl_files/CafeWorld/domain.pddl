(define (domain CafeWorld)
(:requirements :strips :typing :equality :conditional-effects :existential-preconditions :universal-preconditions)
(:types 
	freight
	can
	gripper
	surface
)

(:constants 
	goalLoc_Const - goalLoc
 )

(:predicates 
	(freight_surface_0 ?x - freight ?y - surface)
	(freight_surface_1 ?x - freight ?y - surface)
	(gripper_can_0 ?x - gripper ?y - can)
	(gripper_can_1 ?x - gripper ?y - can)
	(gripper_can_2 ?x - gripper ?y - can)
	(freight_gripper_0 ?x - freight ?y - gripper)
	(freight_gripper_1 ?x - freight ?y - gripper)
	(can_surface_0 ?x - can ?y - surface)
	(can_surface_1 ?x - can ?y - surface)
	(freight_can_0 ?x - freight ?y - can)
	(freight_can_1 ?x - freight ?y - can)
	(clear3_gripper_can_1 ?x - gripper)
	(clear3_freight_can_1 ?x - freight)
	(clear3_freight_gripper_1 ?x - freight)
	(clear3_freight_surface_1 ?x - freight)
	(clear3_can_surface_1 ?x - can)
	(clear3_gripper_can_2 ?x - gripper)
)

(:action a1 
:parameters ( ?can_p1 - can  ?freight_p1 - freight  ?surface_extra_p1 - surface  ?gripper_p1 - gripper )
:precondition (and 
	(can_surface_1 ?can_p1 ?surface_extra_p1)
	(freight_can_0 ?freight_p1 ?can_p1)
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(freight_surface_1 ?freight_p1 ?surface_extra_p1)
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

(:action a2 
:parameters ( ?can_p1 - can  ?gripper_p1 - gripper  ?surface_extra_p1 - surface  ?freight_p1 - freight )
:precondition (and 
	(freight_can_0 ?freight_p1 ?can_p1)
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(freight_surface_1 ?freight_p1 ?surface_extra_p1)
	(gripper_can_2 ?gripper_p1 ?can_p1)
	(clear3_freight_can_1 ?freight_p1) 
	(clear3_freight_gripper_1 ?freight_p1) 
) 
:effect (and 
 	(freight_can_1 ?freight_p1 ?can_p1) 
	(freight_gripper_1 ?freight_p1 ?gripper_p1) 
	(not (freight_can_0 ?freight_p1 ?can_p1))
	(not (freight_gripper_0 ?freight_p1 ?gripper_p1))
	(not (clear3_freight_can_1 ?freight_p1)) 
	(not (clear3_freight_gripper_1 ?freight_p1)) 
 ) 
)

(:action a3 
:parameters ( ?can_p1 - can  ?gripper_p1 - gripper  ?surface_extra_p1 - surface  ?freight_p1 - freight )
:precondition (and 
	(can_surface_1 ?can_p1 ?surface_extra_p1)
	(freight_can_0 ?freight_p1 ?can_p1)
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(freight_surface_1 ?freight_p1 ?surface_extra_p1)
	(gripper_can_1 ?gripper_p1 ?can_p1)
	(clear3_gripper_can_2 ?gripper_p1) 
) 
:effect (and 
 	(gripper_can_0 ?gripper_p1 ?can_p1) 
	(not (gripper_can_1 ?gripper_p1 ?can_p1))
	(not (gripper_can_2 ?gripper_p1 ?can_p1))
	(clear3_gripper_can_1 ?gripper_p1) 
 ) 
)

(:action a4 
:parameters ( ?gripper_p1 - gripper  ?surface_extra_p1 - surface  ?freight_p1 - freight )
:precondition (and 
	(freight_gripper_1 ?freight_p1 ?gripper_p1)
	(freight_surface_1 ?freight_p1 ?surface_extra_p1)
) 
:effect (and 
 	(freight_gripper_0 ?freight_p1 ?gripper_p1) 
	(not (freight_gripper_1 ?freight_p1 ?gripper_p1))
	(clear3_freight_gripper_1 ?freight_p1) 
 ) 
)

(:action a5 
:parameters ( ?gripper_extra_p1 - gripper  ?can_p1 - can  ?freight_extra_p1 - freight  ?surface_p1 - surface )
:precondition (and 
	(can_surface_0 ?can_p1 ?surface_p1)
	(freight_can_0 ?freight_extra_p1 ?can_p1)
	(freight_gripper_0 ?freight_extra_p1 ?gripper_extra_p1)
	(freight_surface_1 ?freight_extra_p1 ?surface_p1)
	(gripper_can_2 ?gripper_extra_p1 ?can_p1)
) 
:effect (and 
 	(can_surface_1 ?can_p1 ?surface_p1) 
	(not (can_surface_0 ?can_p1 ?surface_p1))
	(not (clear3_can_surface_1 ?can_p1)) 
 ) 
)

(:action a6 
:parameters ( ?can_p1 - can  ?gripper_p1 - gripper  ?surface_extra_p1 - surface  ?freight_p1 - freight )
:precondition (and 
	(freight_can_1 ?freight_p1 ?can_p1)
	(freight_gripper_1 ?freight_p1 ?gripper_p1)
	(freight_surface_1 ?freight_p1 ?surface_extra_p1)
	(gripper_can_2 ?gripper_p1 ?can_p1)
) 
:effect (and 
 	(freight_can_0 ?freight_p1 ?can_p1) 
	(freight_gripper_0 ?freight_p1 ?gripper_p1) 
	(not (freight_can_1 ?freight_p1 ?can_p1))
	(not (freight_gripper_1 ?freight_p1 ?gripper_p1))
	(clear3_freight_can_1 ?freight_p1) 
	(clear3_freight_gripper_1 ?freight_p1) 
 ) 
)

(:action a7 
:parameters ( ?can_p1 - can  ?gripper_p1 - gripper  ?surface_extra_p1 - surface  ?freight_p1 - freight )
:precondition (and 
	(can_surface_1 ?can_p1 ?surface_extra_p1)
	(freight_can_0 ?freight_p1 ?can_p1)
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(freight_surface_1 ?freight_p1 ?surface_extra_p1)
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

(:action a8 
:parameters ( ?gripper_p1 - gripper  ?surface_extra_p1 - surface  ?freight_p1 - freight )
:precondition (and 
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(freight_surface_1 ?freight_p1 ?surface_extra_p1)
	(clear3_freight_gripper_1 ?freight_p1) 
) 
:effect (and 
 	(freight_gripper_1 ?freight_p1 ?gripper_p1) 
	(not (freight_gripper_0 ?freight_p1 ?gripper_p1))
	(not (clear3_freight_gripper_1 ?freight_p1)) 
 ) 
)

(:action a9 
:parameters ( ?can_p1 - can  ?freight_p1 - freight  ?surface_extra_p1 - surface  ?gripper_p1 - gripper )
:precondition (and 
	(can_surface_1 ?can_p1 ?surface_extra_p1)
	(freight_can_0 ?freight_p1 ?can_p1)
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(freight_surface_1 ?freight_p1 ?surface_extra_p1)
	(gripper_can_0 ?gripper_p1 ?can_p1)
	(clear3_gripper_can_1 ?gripper_p1) 
	(clear3_gripper_can_2 ?gripper_p1) 
) 
:effect (and 
 	(gripper_can_1 ?gripper_p1 ?can_p1) 
	(not (gripper_can_0 ?gripper_p1 ?can_p1))
	(not (gripper_can_2 ?gripper_p1 ?can_p1))
	(not (clear3_gripper_can_1 ?gripper_p1)) 
 ) 
)

(:action a10 
:parameters ( ?gripper_p1 - gripper  ?surface_p1 - surface  ?freight_p1 - freight )
:precondition (and 
	(freight_gripper_1 ?freight_p1 ?gripper_p1)
	(freight_surface_0 ?freight_p1 ?surface_p1)
	(clear3_freight_surface_1 ?freight_p1) 
) 
:effect (and 
 	(freight_surface_1 ?freight_p1 ?surface_p1) 
	(not (freight_surface_0 ?freight_p1 ?surface_p1))
	(not (clear3_freight_surface_1 ?freight_p1)) 
 ) 
)

(:action a11 
:parameters ( ?gripper_extra_p1 - gripper  ?can_p1 - can  ?freight_extra_p1 - freight  ?surface_p1 - surface )
:precondition (and 
	(can_surface_1 ?can_p1 ?surface_p1)
	(freight_can_0 ?freight_extra_p1 ?can_p1)
	(freight_gripper_0 ?freight_extra_p1 ?gripper_extra_p1)
	(freight_surface_1 ?freight_extra_p1 ?surface_p1)
	(gripper_can_2 ?gripper_extra_p1 ?can_p1)
) 
:effect (and 
 	(can_surface_0 ?can_p1 ?surface_p1) 
	(not (can_surface_1 ?can_p1 ?surface_p1))
	(clear3_can_surface_1 ?can_p1) 
 ) 
)

(:action a12 
:parameters ( ?gripper_p1 - gripper  ?surface_p1 - surface  ?freight_p1 - freight )
:precondition (and 
	(freight_gripper_1 ?freight_p1 ?gripper_p1)
	(freight_surface_1 ?freight_p1 ?surface_p1)
) 
:effect (and 
 	(freight_surface_0 ?freight_p1 ?surface_p1) 
	(not (freight_surface_1 ?freight_p1 ?surface_p1))
	(clear3_freight_surface_1 ?freight_p1) 
 ) 
)


)