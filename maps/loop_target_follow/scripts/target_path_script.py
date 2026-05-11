"""Kinematics script for moving a target sign around the modified map loop"""
""" Written for SIT 310 - DLAI 22/3/26 """
""" v1.0 Kinematics for a road sign going around the modified map, using FSM for road segments """

import numpy as np

from packages.duckiematrix_engine.entities.matrix_entity import (
    MatrixEntityBehavior,
)


class StraightLineScript(MatrixEntityBehavior):
    """Straight line script."""

    _distance: float
    _max_horizontal_distance: float 
    _distance_on_leg: float
    _speed: float
    _direction: float
    _state: int 

    def __init__(
        self,
        matrix_key: str,
        world_key: str | None,
        distance: float = 0.3,
        speed: float = 0.1,
        direction: float = 1.0, 
        max_horizontal_distance: float= 2.0,  
        max_vertical_distance: float=1.0,
        max_curve_distance: float = 0.3, 
        FSM: int = 0,
        sum_angle =0,
    ) -> None:
        """Initialize straight line script."""
        super().__init__(matrix_key, world_key)
        self._distance = 0
        self._distance_on_leg = 0
        self._direction = 1.0
        self._speed = speed
        self._max_horizontal_distance = 1.45 
        self._max_vertical_distance= 0.75 
        self._max_curve_distance= 0.3 
        self._FSM = 0
        self._sum_angle= 0
        

    def update(self, delta_time: float) -> None:
        """Update."""
        if self.state:
            distance = self._speed * delta_time 
            if self._FSM == 0:
                self.state.x += self._direction * distance 
                self._distance_on_leg += distance            
                if self._distance_on_leg > self._max_horizontal_distance:
                    self._FSM = 1 
                    self._distance_on_leg = 0       
            if self._FSM == 1:
                self._sum_angle += 0.45 * delta_time
                self.state.yaw += 0.45 * delta_time
                self.state.x += self._direction * distance * np.cos(self.state.yaw) 
                self.state.y += self._direction * distance * np.sin(self.state.yaw) 
                self._distance_on_leg += distance            
                if self._sum_angle > 1.5:
                    self.state.yaw= 1.5
                    self._FSM = 2 
                    self._sum_angle = 0
                    self._distance_on_leg = 0
            if self._FSM == 2:
                self.state.y += self._direction * distance 
                self._distance_on_leg += distance            
                if self._distance_on_leg > self._max_vertical_distance:
                    self._FSM = 3 
                    self._distance_on_leg = 0
            if self._FSM == 3:
                self._sum_angle += 0.45 * delta_time
                self.state.yaw += 0.45 * delta_time 
                self.state.x += self._direction * distance * np.cos(self.state.yaw) 
                self.state.y += self._direction * distance * np.sin(self.state.yaw) 
                self._distance_on_leg += distance            
                if self._sum_angle > 1.5 :
                    self._FSM = 4
                    self.state.yaw= 3.142
                    self._direction = self._direction * -1 
                    self._sum_angle = 0
                    self._distance_on_leg = 0
            if self._FSM == 4:
                self.state.x += self._direction * distance 
                self._distance_on_leg += distance            
                if self._distance_on_leg > self._max_horizontal_distance:  
                    self._FSM = 5 
                    self._distance_on_leg = 0
                    self._direction = self._direction * -1 
            if self._FSM == 5:
                self._sum_angle += 0.45 * delta_time
                self.state.yaw += 0.45 * delta_time 
                self.state.x += self._direction * distance * np.cos(self.state.yaw) 
                self.state.y += self._direction * distance * np.sin(self.state.yaw) 
                self._distance_on_leg += distance            
                if self._sum_angle > 1.5 :
                    self._FSM = 6
                    self.state.yaw= 4.642
                    self._direction = self._direction * -1 
                    self._sum_angle = 0
                    self._distance_on_leg = 0
            if self._FSM == 6:
                self.state.y += self._direction * distance 
                self._distance_on_leg += distance            
                if self._distance_on_leg > self._max_vertical_distance:
                    self._FSM = 7 
                    self._distance_on_leg = 0
                    self._direction = self._direction * -1
            if self._FSM == 7:
                self._sum_angle += 0.45 * delta_time
                self.state.yaw += 0.45 * delta_time 
                self.state.x += self._direction * distance * np.cos(self.state.yaw) 
                self.state.y += self._direction * distance * np.sin(self.state.yaw) 
                self._distance_on_leg += distance            
                if self._sum_angle > 1.5 :
                    self.state.yaw = 0.0 
                    self._FSM = 0
                    self._direction = 1 
                    self._sum_angle = 0
                    self._distance_on_leg = 0
                    
                    
            self.state.commit()
