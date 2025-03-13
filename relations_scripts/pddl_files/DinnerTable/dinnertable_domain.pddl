(define (domain DinnerTable)
(:requirements :strips :typing :equality :conditional-effects :existential-preconditions :universal-preconditions)
(:types 
	freight
	gripper
	glass
	bowl
	surface
	glasstargetLoc
	glassinitLoc
	bowltargetLoc
	bowlinitLoc
)

(:constants 
 )

(:predicates 
	(glass_glasstargetLoc_0 ?x - glass ?y - glasstargetLoc)
	(glass_glasstargetLoc_1 ?x - glass ?y - glasstargetLoc)
	(freight_bowl_0 ?x - freight ?y - bowl)
	(freight_bowlinitLoc_0 ?x - freight ?y - bowlinitLoc)
	(freight_bowlinitLoc_1 ?x - freight ?y - bowlinitLoc)
	(bowl_bowlinitLoc_0 ?x - bowl ?y - bowlinitLoc)
	(bowl_bowlinitLoc_1 ?x - bowl ?y - bowlinitLoc)
	(gripper_bowl_0 ?x - gripper ?y - bowl)
	(gripper_bowl_1 ?x - gripper ?y - bowl)
	(gripper_bowl_2 ?x - gripper ?y - bowl)
	(freight_bowltargetLoc_0 ?x - freight ?y - bowltargetLoc)
	(freight_bowltargetLoc_1 ?x - freight ?y - bowltargetLoc)
	(freight_glass_0 ?x - freight ?y - glass)
	(bowl_bowltargetLoc_0 ?x - bowl ?y - bowltargetLoc)
	(bowl_bowltargetLoc_1 ?x - bowl ?y - bowltargetLoc)
	(glass_glassinitLoc_0 ?x - glass ?y - glassinitLoc)
	(glass_glassinitLoc_1 ?x - glass ?y - glassinitLoc)
	(freight_gripper_0 ?x - freight ?y - gripper)
	(freight_gripper_1 ?x - freight ?y - gripper)
	(freight_glassinitLoc_0 ?x - freight ?y - glassinitLoc)
	(freight_glassinitLoc_1 ?x - freight ?y - glassinitLoc)
	(freight_glasstargetLoc_0 ?x - freight ?y - glasstargetLoc)
	(freight_glasstargetLoc_1 ?x - freight ?y - glasstargetLoc)
	(gripper_glass_0 ?x - gripper ?y - glass)
	(gripper_glass_1 ?x - gripper ?y - glass)
	(gripper_glass_2 ?x - gripper ?y - glass)
	(clear3_freight_bowltargetLoc_1 ?x - freight)
	(clear3_gripper_glass_1 ?x - gripper)
	(clear3_gripper_bowl_2 ?x - gripper)
	(clear3_glass_glasstargetLoc_1 ?x - glass)
	(clear3_gripper_bowl_1 ?x - gripper)
	(clear3_gripper_glass_2 ?x - gripper)
	(clear3_freight_glasstargetLoc_1 ?x - freight)
	(clear3_freight_bowlinitLoc_1 ?x - freight)
	(clear3_bowl_bowltargetLoc_1 ?x - bowl)
	(clear3_freight_glassinitLoc_1 ?x - freight)
	(clear3_freight_gripper_1 ?x - freight)
	(clear3_bowl_bowlinitLoc_1 ?x - bowl)
	(clear3_glass_glassinitLoc_1 ?x - glass)
)

(:action a1 
:parameters ( ?bowl_extra_p1 - bowl  ?freight_p1 - freight  ?bowlinitLoc_p1 - bowlinitLoc  ?gripper_p1 - gripper )
:precondition (and 
	(bowl_bowlinitLoc_0 ?bowl_extra_p1 ?bowlinitLoc_p1)
	(freight_bowlinitLoc_1 ?freight_p1 ?bowlinitLoc_p1)
	(freight_gripper_1 ?freight_p1 ?gripper_p1)
	(gripper_bowl_2 ?gripper_p1 ?bowl_extra_p1)
) 
:effect (and 
 	(freight_bowlinitLoc_0 ?freight_p1 ?bowlinitLoc_p1) 
	(not (freight_bowlinitLoc_1 ?freight_p1 ?bowlinitLoc_p1))
	(clear3_freight_bowlinitLoc_1 ?freight_p1) 
 ) 
)

(:action a2 
:parameters ( ?glassinitLoc_extra_p1 - glassinitLoc  ?gripper_p1 - gripper  ?glasstargetLoc_p1 - glasstargetLoc  ?freight_p1 - freight )
:precondition (and 
	(freight_glassinitLoc_0 ?freight_p1 ?glassinitLoc_extra_p1)
	(freight_glasstargetLoc_1 ?freight_p1 ?glasstargetLoc_p1)
	(freight_gripper_1 ?freight_p1 ?gripper_p1)
) 
:effect (and 
 	(freight_glasstargetLoc_0 ?freight_p1 ?glasstargetLoc_p1) 
	(not (freight_glasstargetLoc_1 ?freight_p1 ?glasstargetLoc_p1))
	(clear3_freight_glasstargetLoc_1 ?freight_p1) 
 ) 
)

(:action a3 
:parameters ( ?bowl_p1 - bowl  ?gripper_extra_p1 - gripper  ?freight_extra_p1 - freight  ?bowlinitLoc_p1 - bowlinitLoc )
:precondition (and 
	(bowl_bowlinitLoc_1 ?bowl_p1 ?bowlinitLoc_p1)
	(freight_bowl_0 ?freight_extra_p1 ?bowl_p1)
	(freight_bowlinitLoc_1 ?freight_extra_p1 ?bowlinitLoc_p1)
	(freight_gripper_0 ?freight_extra_p1 ?gripper_extra_p1)
	(gripper_bowl_2 ?gripper_extra_p1 ?bowl_p1)
) 
:effect (and 
 	(bowl_bowlinitLoc_0 ?bowl_p1 ?bowlinitLoc_p1) 
	(not (bowl_bowlinitLoc_1 ?bowl_p1 ?bowlinitLoc_p1))
	(clear3_bowl_bowlinitLoc_1 ?bowl_p1) 
 ) 
)

(:action a4 
:parameters ( ?gripper_p1 - gripper  ?bowltargetLoc_p1 - bowltargetLoc  ?bowlinitLoc_extra_p1 - bowlinitLoc  ?freight_p1 - freight )
:precondition (and 
	(freight_bowlinitLoc_0 ?freight_p1 ?bowlinitLoc_extra_p1)
	(freight_bowltargetLoc_1 ?freight_p1 ?bowltargetLoc_p1)
	(freight_gripper_1 ?freight_p1 ?gripper_p1)
) 
:effect (and 
 	(freight_bowltargetLoc_0 ?freight_p1 ?bowltargetLoc_p1) 
	(not (freight_bowltargetLoc_1 ?freight_p1 ?bowltargetLoc_p1))
	(clear3_freight_bowltargetLoc_1 ?freight_p1) 
 ) 
)

(:action a5 
:parameters ( ?bowl_p1 - bowl  ?gripper_extra_p1 - gripper  ?bowlinitLoc_extra_p1 - bowlinitLoc  ?freight_extra_p1 - freight  ?bowltargetLoc_p1 - bowltargetLoc )
:precondition (and 
	(bowl_bowlinitLoc_0 ?bowl_p1 ?bowlinitLoc_extra_p1)
	(bowl_bowltargetLoc_0 ?bowl_p1 ?bowltargetLoc_p1)
	(freight_bowl_0 ?freight_extra_p1 ?bowl_p1)
	(freight_bowlinitLoc_0 ?freight_extra_p1 ?bowlinitLoc_extra_p1)
	(freight_bowltargetLoc_1 ?freight_extra_p1 ?bowltargetLoc_p1)
	(freight_gripper_0 ?freight_extra_p1 ?gripper_extra_p1)
	(gripper_bowl_2 ?gripper_extra_p1 ?bowl_p1)
	(clear3_bowl_bowltargetLoc_1 ?bowl_p1) 
) 
:effect (and 
 	(bowl_bowltargetLoc_1 ?bowl_p1 ?bowltargetLoc_p1) 
	(not (bowl_bowltargetLoc_0 ?bowl_p1 ?bowltargetLoc_p1))
	(not (clear3_bowl_bowltargetLoc_1 ?bowl_p1)) 
 ) 
)

(:action a6 
:parameters ( ?glasstargetLoc_extra_p1 - glasstargetLoc  ?glass_p1 - glass  ?gripper_p1 - gripper  ?glassinitLoc_extra_p1 - glassinitLoc  ?freight_p1 - freight )
:precondition (and 
	(freight_glass_0 ?freight_p1 ?glass_p1)
	(freight_glassinitLoc_0 ?freight_p1 ?glassinitLoc_extra_p1)
	(freight_glasstargetLoc_1 ?freight_p1 ?glasstargetLoc_extra_p1)
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(glass_glassinitLoc_0 ?glass_p1 ?glassinitLoc_extra_p1)
	(glass_glasstargetLoc_1 ?glass_p1 ?glasstargetLoc_extra_p1)
	(gripper_glass_2 ?gripper_p1 ?glass_p1)
	(clear3_gripper_glass_1 ?gripper_p1) 
) 
:effect (and 
 	(gripper_glass_1 ?gripper_p1 ?glass_p1) 
	(not (gripper_glass_0 ?gripper_p1 ?glass_p1))
	(not (gripper_glass_2 ?gripper_p1 ?glass_p1))
	(clear3_gripper_glass_2 ?gripper_p1) 
	(not (clear3_gripper_glass_1 ?gripper_p1)) 
 ) 
)

(:action a7 
:parameters ( ?bowltargetLoc_extra_p1 - bowltargetLoc  ?bowl_p1 - bowl  ?gripper_p1 - gripper  ?bowlinitLoc_extra_p1 - bowlinitLoc  ?freight_p1 - freight )
:precondition (and 
	(bowl_bowlinitLoc_0 ?bowl_p1 ?bowlinitLoc_extra_p1)
	(bowl_bowltargetLoc_1 ?bowl_p1 ?bowltargetLoc_extra_p1)
	(freight_bowl_0 ?freight_p1 ?bowl_p1)
	(freight_bowlinitLoc_0 ?freight_p1 ?bowlinitLoc_extra_p1)
	(freight_bowltargetLoc_1 ?freight_p1 ?bowltargetLoc_extra_p1)
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(gripper_bowl_1 ?gripper_p1 ?bowl_p1)
	(clear3_gripper_bowl_2 ?gripper_p1) 
) 
:effect (and 
 	(gripper_bowl_0 ?gripper_p1 ?bowl_p1) 
	(not (gripper_bowl_1 ?gripper_p1 ?bowl_p1))
	(not (gripper_bowl_2 ?gripper_p1 ?bowl_p1))
	(clear3_gripper_bowl_1 ?gripper_p1) 
 ) 
)

(:action a8 
:parameters ( ?bowltargetLoc_extra_p1 - bowltargetLoc  ?bowl_p1 - bowl  ?gripper_p1 - gripper  ?bowlinitLoc_extra_p1 - bowlinitLoc  ?freight_p1 - freight )
:precondition (and 
	(bowl_bowlinitLoc_0 ?bowl_p1 ?bowlinitLoc_extra_p1)
	(bowl_bowltargetLoc_1 ?bowl_p1 ?bowltargetLoc_extra_p1)
	(freight_bowl_0 ?freight_p1 ?bowl_p1)
	(freight_bowlinitLoc_0 ?freight_p1 ?bowlinitLoc_extra_p1)
	(freight_bowltargetLoc_1 ?freight_p1 ?bowltargetLoc_extra_p1)
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(gripper_bowl_2 ?gripper_p1 ?bowl_p1)
	(clear3_gripper_bowl_1 ?gripper_p1) 
) 
:effect (and 
 	(gripper_bowl_1 ?gripper_p1 ?bowl_p1) 
	(not (gripper_bowl_0 ?gripper_p1 ?bowl_p1))
	(not (gripper_bowl_2 ?gripper_p1 ?bowl_p1))
	(clear3_gripper_bowl_2 ?gripper_p1) 
	(not (clear3_gripper_bowl_1 ?gripper_p1)) 
 ) 
)

(:action a9 
:parameters ( ?glasstargetLoc_p1 - glasstargetLoc  ?glass_p1 - glass  ?freight_extra_p1 - freight  ?glassinitLoc_extra_p1 - glassinitLoc  ?gripper_extra_p1 - gripper )
:precondition (and 
	(freight_glass_0 ?freight_extra_p1 ?glass_p1)
	(freight_glassinitLoc_0 ?freight_extra_p1 ?glassinitLoc_extra_p1)
	(freight_glasstargetLoc_1 ?freight_extra_p1 ?glasstargetLoc_p1)
	(freight_gripper_0 ?freight_extra_p1 ?gripper_extra_p1)
	(glass_glassinitLoc_0 ?glass_p1 ?glassinitLoc_extra_p1)
	(glass_glasstargetLoc_0 ?glass_p1 ?glasstargetLoc_p1)
	(gripper_glass_2 ?gripper_extra_p1 ?glass_p1)
	(clear3_glass_glasstargetLoc_1 ?glass_p1) 
) 
:effect (and 
 	(glass_glasstargetLoc_1 ?glass_p1 ?glasstargetLoc_p1) 
	(not (glass_glasstargetLoc_0 ?glass_p1 ?glasstargetLoc_p1))
	(not (clear3_glass_glasstargetLoc_1 ?glass_p1)) 
 ) 
)

(:action a10 
:parameters ( ?bowltargetLoc_p1 - bowltargetLoc  ?bowl_extra_p1 - bowl  ?gripper_p1 - gripper  ?bowlinitLoc_extra_p1 - bowlinitLoc  ?freight_p1 - freight )
:precondition (and 
	(bowl_bowlinitLoc_0 ?bowl_extra_p1 ?bowlinitLoc_extra_p1)
	(freight_bowlinitLoc_0 ?freight_p1 ?bowlinitLoc_extra_p1)
	(freight_bowltargetLoc_0 ?freight_p1 ?bowltargetLoc_p1)
	(freight_gripper_1 ?freight_p1 ?gripper_p1)
	(gripper_bowl_2 ?gripper_p1 ?bowl_extra_p1)
	(clear3_freight_bowltargetLoc_1 ?freight_p1) 
) 
:effect (and 
 	(freight_bowltargetLoc_1 ?freight_p1 ?bowltargetLoc_p1) 
	(not (freight_bowltargetLoc_0 ?freight_p1 ?bowltargetLoc_p1))
	(not (clear3_freight_bowltargetLoc_1 ?freight_p1)) 
 ) 
)

(:action a11 
:parameters ( ?glassinitLoc_p1 - glassinitLoc  ?gripper_p1 - gripper  ?freight_p1 - freight )
:precondition (and 
	(freight_glassinitLoc_0 ?freight_p1 ?glassinitLoc_p1)
	(freight_gripper_1 ?freight_p1 ?gripper_p1)
	(clear3_freight_glassinitLoc_1 ?freight_p1) 
) 
:effect (and 
 	(freight_glassinitLoc_1 ?freight_p1 ?glassinitLoc_p1) 
	(not (freight_glassinitLoc_0 ?freight_p1 ?glassinitLoc_p1))
	(not (clear3_freight_glassinitLoc_1 ?freight_p1)) 
 ) 
)

(:action a12 
:parameters ( ?bowl_p1 - bowl  ?gripper_p1 - gripper  ?bowlinitLoc_extra_p1 - bowlinitLoc  ?freight_p1 - freight )
:precondition (and 
	(bowl_bowlinitLoc_1 ?bowl_p1 ?bowlinitLoc_extra_p1)
	(freight_bowl_0 ?freight_p1 ?bowl_p1)
	(freight_bowlinitLoc_1 ?freight_p1 ?bowlinitLoc_extra_p1)
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(gripper_bowl_1 ?gripper_p1 ?bowl_p1)
	(clear3_gripper_bowl_2 ?gripper_p1) 
) 
:effect (and 
 	(gripper_bowl_2 ?gripper_p1 ?bowl_p1) 
	(not (gripper_bowl_0 ?gripper_p1 ?bowl_p1))
	(not (gripper_bowl_1 ?gripper_p1 ?bowl_p1))
	(clear3_gripper_bowl_1 ?gripper_p1) 
	(not (clear3_gripper_bowl_2 ?gripper_p1)) 
 ) 
)

(:action a13 
:parameters ( ?gripper_p1 - gripper  ?bowl_p1 - bowl  ?bowlinitLoc_extra_p1 - bowlinitLoc  ?freight_p1 - freight )
:precondition (and 
	(bowl_bowlinitLoc_1 ?bowl_p1 ?bowlinitLoc_extra_p1)
	(freight_bowl_0 ?freight_p1 ?bowl_p1)
	(freight_bowlinitLoc_1 ?freight_p1 ?bowlinitLoc_extra_p1)
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(gripper_bowl_0 ?gripper_p1 ?bowl_p1)
	(clear3_gripper_bowl_1 ?gripper_p1) 
	(clear3_gripper_bowl_2 ?gripper_p1) 
) 
:effect (and 
 	(gripper_bowl_1 ?gripper_p1 ?bowl_p1) 
	(not (gripper_bowl_0 ?gripper_p1 ?bowl_p1))
	(not (gripper_bowl_2 ?gripper_p1 ?bowl_p1))
	(not (clear3_gripper_bowl_1 ?gripper_p1)) 
 ) 
)

(:action a14 
:parameters ( ?glasstargetLoc_extra_p1 - glasstargetLoc  ?glass_p1 - glass  ?freight_p1 - freight  ?glassinitLoc_extra_p1 - glassinitLoc  ?gripper_p1 - gripper )
:precondition (and 
	(freight_glass_0 ?freight_p1 ?glass_p1)
	(freight_glassinitLoc_0 ?freight_p1 ?glassinitLoc_extra_p1)
	(freight_glasstargetLoc_1 ?freight_p1 ?glasstargetLoc_extra_p1)
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(glass_glassinitLoc_0 ?glass_p1 ?glassinitLoc_extra_p1)
	(glass_glasstargetLoc_1 ?glass_p1 ?glasstargetLoc_extra_p1)
	(gripper_glass_1 ?gripper_p1 ?glass_p1)
	(clear3_gripper_glass_2 ?gripper_p1) 
) 
:effect (and 
 	(gripper_glass_0 ?gripper_p1 ?glass_p1) 
	(not (gripper_glass_1 ?gripper_p1 ?glass_p1))
	(not (gripper_glass_2 ?gripper_p1 ?glass_p1))
	(clear3_gripper_glass_1 ?gripper_p1) 
 ) 
)

(:action a15 
:parameters ( ?glasstargetLoc_p1 - glasstargetLoc  ?glassinitLoc_extra_p1 - glassinitLoc  ?freight_p1 - freight  ?glass_extra_p1 - glass  ?gripper_p1 - gripper )
:precondition (and 
	(freight_glassinitLoc_0 ?freight_p1 ?glassinitLoc_extra_p1)
	(freight_glasstargetLoc_0 ?freight_p1 ?glasstargetLoc_p1)
	(freight_gripper_1 ?freight_p1 ?gripper_p1)
	(glass_glassinitLoc_0 ?glass_extra_p1 ?glassinitLoc_extra_p1)
	(gripper_glass_2 ?gripper_p1 ?glass_extra_p1)
	(clear3_freight_glasstargetLoc_1 ?freight_p1) 
) 
:effect (and 
 	(freight_glasstargetLoc_1 ?freight_p1 ?glasstargetLoc_p1) 
	(not (freight_glasstargetLoc_0 ?freight_p1 ?glasstargetLoc_p1))
	(not (clear3_freight_glasstargetLoc_1 ?freight_p1)) 
 ) 
)

(:action a16 
:parameters ( ?gripper_p1 - gripper  ?freight_p1 - freight )
:precondition (and 
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(clear3_freight_gripper_1 ?freight_p1) 
) 
:effect (and 
 	(freight_gripper_1 ?freight_p1 ?gripper_p1) 
	(not (freight_gripper_0 ?freight_p1 ?gripper_p1))
	(not (clear3_freight_gripper_1 ?freight_p1)) 
 ) 
)

(:action a17 
:parameters ( ?glassinitLoc_extra_p1 - glassinitLoc  ?gripper_p1 - gripper  ?glass_p1 - glass  ?freight_p1 - freight )
:precondition (and 
	(freight_glass_0 ?freight_p1 ?glass_p1)
	(freight_glassinitLoc_1 ?freight_p1 ?glassinitLoc_extra_p1)
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(glass_glassinitLoc_1 ?glass_p1 ?glassinitLoc_extra_p1)
	(gripper_glass_1 ?gripper_p1 ?glass_p1)
	(clear3_gripper_glass_2 ?gripper_p1) 
) 
:effect (and 
 	(gripper_glass_2 ?gripper_p1 ?glass_p1) 
	(not (gripper_glass_0 ?gripper_p1 ?glass_p1))
	(not (gripper_glass_1 ?gripper_p1 ?glass_p1))
	(clear3_gripper_glass_1 ?gripper_p1) 
	(not (clear3_gripper_glass_2 ?gripper_p1)) 
 ) 
)

(:action a18 
:parameters ( ?glassinitLoc_p1 - glassinitLoc  ?glass_p1 - glass  ?freight_extra_p1 - freight  ?gripper_extra_p1 - gripper )
:precondition (and 
	(freight_glass_0 ?freight_extra_p1 ?glass_p1)
	(freight_glassinitLoc_1 ?freight_extra_p1 ?glassinitLoc_p1)
	(freight_gripper_0 ?freight_extra_p1 ?gripper_extra_p1)
	(glass_glassinitLoc_1 ?glass_p1 ?glassinitLoc_p1)
	(gripper_glass_2 ?gripper_extra_p1 ?glass_p1)
) 
:effect (and 
 	(glass_glassinitLoc_0 ?glass_p1 ?glassinitLoc_p1) 
	(not (glass_glassinitLoc_1 ?glass_p1 ?glassinitLoc_p1))
	(clear3_glass_glassinitLoc_1 ?glass_p1) 
 ) 
)

(:action a19 
:parameters ( ?gripper_p1 - gripper  ?freight_p1 - freight )
:precondition (and 
	(freight_gripper_1 ?freight_p1 ?gripper_p1)
) 
:effect (and 
 	(freight_gripper_0 ?freight_p1 ?gripper_p1) 
	(not (freight_gripper_1 ?freight_p1 ?gripper_p1))
	(clear3_freight_gripper_1 ?freight_p1) 
 ) 
)

(:action a20 
:parameters ( ?glassinitLoc_p1 - glassinitLoc  ?freight_p1 - freight  ?glass_extra_p1 - glass  ?gripper_p1 - gripper )
:precondition (and 
	(freight_glassinitLoc_1 ?freight_p1 ?glassinitLoc_p1)
	(freight_gripper_1 ?freight_p1 ?gripper_p1)
	(glass_glassinitLoc_0 ?glass_extra_p1 ?glassinitLoc_p1)
	(gripper_glass_2 ?gripper_p1 ?glass_extra_p1)
) 
:effect (and 
 	(freight_glassinitLoc_0 ?freight_p1 ?glassinitLoc_p1) 
	(not (freight_glassinitLoc_1 ?freight_p1 ?glassinitLoc_p1))
	(clear3_freight_glassinitLoc_1 ?freight_p1) 
 ) 
)

(:action a21 
:parameters ( ?glassinitLoc_extra_p1 - glassinitLoc  ?gripper_p1 - gripper  ?glass_p1 - glass  ?freight_p1 - freight )
:precondition (and 
	(freight_glass_0 ?freight_p1 ?glass_p1)
	(freight_glassinitLoc_1 ?freight_p1 ?glassinitLoc_extra_p1)
	(freight_gripper_0 ?freight_p1 ?gripper_p1)
	(glass_glassinitLoc_1 ?glass_p1 ?glassinitLoc_extra_p1)
	(gripper_glass_0 ?gripper_p1 ?glass_p1)
	(clear3_gripper_glass_1 ?gripper_p1) 
	(clear3_gripper_glass_2 ?gripper_p1) 
) 
:effect (and 
 	(gripper_glass_1 ?gripper_p1 ?glass_p1) 
	(not (gripper_glass_0 ?gripper_p1 ?glass_p1))
	(not (gripper_glass_2 ?gripper_p1 ?glass_p1))
	(not (clear3_gripper_glass_1 ?gripper_p1)) 
 ) 
)

(:action a22 
:parameters ( ?gripper_p1 - gripper  ?bowlinitLoc_p1 - bowlinitLoc  ?freight_p1 - freight )
:precondition (and 
	(freight_bowlinitLoc_0 ?freight_p1 ?bowlinitLoc_p1)
	(freight_gripper_1 ?freight_p1 ?gripper_p1)
	(clear3_freight_bowlinitLoc_1 ?freight_p1) 
) 
:effect (and 
 	(freight_bowlinitLoc_1 ?freight_p1 ?bowlinitLoc_p1) 
	(not (freight_bowlinitLoc_0 ?freight_p1 ?bowlinitLoc_p1))
	(not (clear3_freight_bowlinitLoc_1 ?freight_p1)) 
 ) 
)


)