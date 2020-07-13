#!/usr/bin/env python
import smach


class FlyToEstimatedTurbinePosition(smach.State):
    def __init__(self, outcomes=['outcome1', 'outcome2']):
        pass
    def execute(self, userdata):
        # Your state execution goes here
        # return self.outcomes[0]
        pass
