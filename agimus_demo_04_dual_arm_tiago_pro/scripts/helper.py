#!/usr/bin/env python
# Copyright (c) 2025 CNRS
# Author: Florent Lamiraux
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

from CORBA import Any, TC_long, TC_double
from math import sqrt

class Helper:
    def __init__(self, ps, graph):
        self.robot = ps.robot
        self.problem = ps.client.basic.problem
        self.graph = graph
        self.problem.resetConstraints()
        # get and create some CORBA objects
        self.cproblem = self.problem.getProblem()
        self.crobot = self.cproblem.robot()
        self.cdistance = self.cproblem.getDistance()
        self.croadmap = self.problem.createRoadmap(self.cdistance, self.crobot)
        self.ctransitionPlanner = self.problem.createPathPlanner("TransitionPlanner", self.cproblem,
            self.croadmap)
        self.ctransitionPlanner.maxIterations(1000)
        self.csplineOptimizer = self.problem.createPathOptimizer(
            "SplineGradientBased_bezier3", self.cproblem
        )
        self.ctimeParameterization = self.problem.createPathOptimizer(
            "SimpleTimeParameterization", self.cproblem
        )
        self.cproblem.setParameter("SimpleTimeParameterization/order", Any(TC_long, 2))
        self.cproblem.setParameter(
            "SimpleTimeParameterization/maxAcceleration", Any(TC_double, 0.2)
        )
        # Create constraint on bar pose (horizontal at .85 meter above the ground
        self.problem.createPositionConstraint("horizontal-bar",
            "", "reinforcment_bar/root_joint",
            [0, 0, 0.85], [0, 0, 0],
            [False, False, True])

    def __del__(self):
        self.cproblem.deleteThis()
        self.crobot.deleteThis()
        self.cdistance.deleteThis()
        self.croadmap.deleteThis()
        self.ctransitionPlanner.deleteThis()
        self.csplineOptimizer.deleteThis()
        self.ctimeParameterization.deleteThis()

    # Generate a preplace configuration from reachable from goal
    def generateIntermediateConfigs(self, q_init, q_goal):
        for i in range(10000):
            q = self.robot.shootRandomConfig()
            # Project random configuration in pregrasp reachable from q_goal -> q8
            edge = "tiago_pro/left > reinforcment_bar/left | f_01"
            res, q8, err = self.graph.generateTargetConfig(edge, q_goal, q)
            if not res:
                continue
            res, _ = self.robot.isConfigValid(q8)
            if not res:
                continue
            # Generate corresponding grasp configuration -> q7
            edge = "tiago_pro/left > reinforcment_bar/left | f_12"
            res, q7, err = self.graph.generateTargetConfig(edge, q8, q8)
            if not res:
                continue
            res, _ = self.robot.isConfigValid(q7)
            # Generate corresponding preplacement configuration -> q6
            edge = "tiago_pro/left > reinforcment_bar/left | f_23"
            res, q6, err = self.graph.generateTargetConfig(edge, q7, q7)
            if not res:
                continue
            res, _ = self.robot.isConfigValid(q6)
            if not res:
                continue
            # Project result on preplacement with place_reinforcment_bar/complement
            # initialized with q_init -> q3
            # Build projector to project a configuration in preplacement with placement
            # complement initialized with q_init
            self.problem.resetConstraints()
            self.problem.addNumericalConstraints(
                "cp",
                [
                    "tiago_pro/left grasps reinforcment_bar/left",
                    "tiago_pro/right grasps reinforcment_bar/right",
                    "preplace_reinforcment_bar",
                    "locked_plate/root_joint",
                    "place_reinforcment_bar/complement",
                ],
                [0, 0, 0, 0, 0],
            )
            self.problem.setRightHandSideFromConfigByName(
                "place_reinforcment_bar/complement", q_init
            )
            self.problem.setRightHandSideFromConfigByName("locked_plate/root_joint", q_init)

            res, q3, err = self.problem.applyConstraints(q6)
            res, _ = self.robot.isConfigValid(q3)
            if not res: continue
            # From q6, generate a configuration where,
            #    - the bar is grasped,
            #    - the bar at .85 m above the ground,
            #    - the robot base is in the same pose as in q6
            #  -> q5
            self.problem.resetConstraints()
            self.problem.addNumericalConstraints("cp",
                ["tiago_pro/left grasps reinforcment_bar/left",
                 "tiago_pro/right grasps reinforcment_bar/right",
                 "locked_plate/root_joint",
                 "locked_tiago_pro/root_joint",
                 "horizontal-bar"],
                                                   [0,0,0,0,0])
            self.problem.setRightHandSideFromConfigByName("locked_plate/root_joint", q_init)
            self.problem.setRightHandSideFromConfigByName("locked_tiago_pro/root_joint", q6)
            res, q5, err = self.problem.applyConstraints(q6)
            if not res: continue
            res, _ = self.robot.isConfigValid(q5)
            if not res: continue
            # Generate a configuration in grasp with q3 base pose and q5 arm and torso
            # configurations -> q4
            res, q4, err = self.graph.generateTargetConfig("Loop | 0-0_arm", q3, q5)
            if not res: continue
            res, _ = self.robot.isConfigValid(q4)
            if not res: continue
            # Generate a grasp placement from q3 -> q2
            res, q2, err = self.graph.generateTargetConfig(
                "tiago_pro/left < reinforcment_bar/left | 0-0_32",
                q3, q3)
            if not res: continue
            res, _ = self.robot.isConfigValid(q2)
            if not res: continue
            # Generate pregrasp from q2 -> q1
            res, q1, err = self.graph.generateTargetConfig(
                "tiago_pro/left < reinforcment_bar/left | 0-0_21",
                q2, q2)
            if not res: continue
            res, _ = self.robot.isConfigValid(q1)
            if not res: continue
            # Generate configuration with base in initial pose and arm in q1 -> q0
            res, q0, err = self.graph.generateTargetConfig("Loop | f_arm", q_init, q1)
            if not res: continue
            res, _ = self.robot.isConfigValid(q0)
            if not res: continue
            # Generate configuration with q_goal base pose and q8 arm config -> q9
            q9 = q8[:]
            r = self.robot.rankInConfiguration["tiago_pro/root_joint"]
            q9[r:r+4] = q_goal[r:r+4]
            if res:
                break
        if not res:
            raise RuntimeError(
                "Failed to generate intermediate configurations to solve the problem."
            )
        self.problem.resetConstraints()
        return q0, q1, q2, q3, q4, q5, q6, q7, q8, q9

    def planPath(self, q_init, q_goal):
        configs = self.generateIntermediateConfigs(q_init, q_goal)
        self.ctransitionPlanner.setEdge(self.graph.edges["Loop | f_arm"])
        p1 = self.ctransitionPlanner.planPath(q_init, [configs[0]], True)
        self.problem.addPath(p1)
        i = 0
        self.ctransitionPlanner.setEdge(self.graph.edges["Loop | f_base"])
        p2 = self.ctransitionPlanner.planPath(configs[i], [configs[i+1]], True)
        self.problem.addPath(p2)
        i+=1
        self.ctransitionPlanner.setEdge(self.graph.edges[
            "tiago_pro/left > reinforcment_bar/left | f_12"])
        p3, res, msg = self.ctransitionPlanner.directPath(configs[i], configs[i+1], False)
        if not res:
            raise RuntimeError(f"Failed to grasp the bar: {msg}")
        self.problem.addPath(p3.asVector())
        i+=1
        self.ctransitionPlanner.setEdge(self.graph.edges[
            "tiago_pro/left > reinforcment_bar/left | f_23"])
        p4, res, msg = self.ctransitionPlanner.directPath(configs[i], configs[i+1], False)
        if not res:
            raise RuntimeError(f"Failed to grasp the bar: {msg}")
        self.problem.addPath(p4.asVector())
        i+=1
        self.ctransitionPlanner.setEdge(self.graph.edges["Loop | 0-0_arm"])
        p5 = self.ctransitionPlanner.planPath(configs[i], [configs[i+1]], True)
        self.problem.addPath(p5)
        i+=1
        self.ctransitionPlanner.setEdge(self.graph.edges["Loop | 0-0_base"])
        p6 = self.ctransitionPlanner.planPath(configs[i], [configs[i+1]], True)
        self.problem.addPath(p6)
        i+=1
        self.ctransitionPlanner.setEdge(self.graph.edges["Loop | 0-0_arm"])
        p7 = self.ctransitionPlanner.planPath(configs[i], [configs[i+1]], True)
        self.problem.addPath(p7)
        i+=1
        self.ctransitionPlanner.setEdge(self.graph.edges[
            "tiago_pro/left < reinforcment_bar/left | 0-0_32"])
        p8, res, msg = self.ctransitionPlanner.directPath(configs[i], configs[i+1], False)
        self.problem.addPath(p8.asVector())
        i+=1
        self.ctransitionPlanner.setEdge(self.graph.edges[
            "tiago_pro/left < reinforcment_bar/left | 0-0_21"])
        p9, res, msg = self.ctransitionPlanner.directPath(configs[i], configs[i+1], False)
        self.problem.addPath(p9.asVector())
        i+=1
        self.ctransitionPlanner.setEdge(self.graph.edges["Loop | f_base"])
        p10 = self.ctransitionPlanner.planPath(configs[i], [configs[i+1]], True)
        self.problem.addPath(p10)
        i+=1
        self.ctransitionPlanner.setEdge(self.graph.edges["Loop | f_arm"])
        p11 = self.ctransitionPlanner.planPath(configs[i], [q_goal], True)
        self.problem.addPath(p11)



    def optimizePath(self, pid):
        # segment paths into transitions
        p1 = self.problem.getPath(pid)
        p2 = p1.flatten()
        segment0 = None
        for i in range(p2.numberPaths()):
            p3 = p2.pathAtRank(i)
            if (
                self.graph.getNode(p3.end())
                == "tiago_pro/left grasps reinforcment_bar/left"
            ):
                break
            if segment0 is None:
                segment0 = p3.asVector()
            else:
                segment0.appendPath(p3)
            p3.deleteThis()
        segment1 = p3.asVector()
        p3.deleteThis()
        p3 = p2.pathAtRank(i + 1)
        segment2 = p3.asVector()
        p3.deleteThis()
        segment3 = None
        for i in range(i + 2, p2.numberPaths()):
            p3 = p2.pathAtRank(i)
            if self.graph.getNode(p3.end()) == "free":
                break
            if segment3 is None:
                segment3 = p3.asVector()
            else:
                segment3.appendPath(p3)
            p3.deleteThis()
        segment4 = p3.asVector()
        p3.deleteThis()
        p3 = p2.pathAtRank(i + 1)
        segment5 = p3.asVector()
        p3.deleteThis()
        segment6 = None
        for i in range(i + 2, p2.numberPaths()):
            p3 = p2.pathAtRank(i)
            if self.graph.getNode(p3.end()) != "free":
                raise RuntimeError(
                    "Failed to optimize path: the robot releases the object"
                    + " more than once"
                )
            if segment6 is None:
                segment6 = p3.asVector()
            else:
                segment6.appendPath(p3)
            p3.deleteThis()
        optim0 = self.csplineOptimizer.optimize(segment0)
        optim1 = self.ctimeParameterization.optimize(segment1)
        optim2 = self.ctimeParameterization.optimize(segment2)
        optim3 = self.ctimeParameterization.optimize(segment3)
        optim4 = self.ctimeParameterization.optimize(segment4)
        optim5 = self.ctimeParameterization.optimize(segment5)
        optim6 = self.csplineOptimizer.optimize(segment6)
        res = optim0.asVector()
        res.concatenate(optim1)
        res.concatenate(optim2)
        res.concatenate(optim3)
        res.concatenate(optim4)
        res.concatenate(optim5)
        res.concatenate(optim6)
        optim0.deleteThis()
        optim1.deleteThis()
        optim2.deleteThis()
        optim3.deleteThis()
        optim4.deleteThis()
        optim5.deleteThis()
        optim6.deleteThis()
        self.problem.addPath(res)
        res.deleteThis()
