/*
 * Copyright (C) 2021 Eric Medvet <eric.medvet@gmail.com> (as Eric Medvet <eric.medvet@gmail.com>)
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package it.units.erallab.hmsrobots;

import it.units.erallab.hmsrobots.core.controllers.DistributedSensing;
import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.tasks.locomotion.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.RobotUtils;
import it.units.erallab.hmsrobots.util.SerializationUtils;
import it.units.erallab.hmsrobots.viewers.GridOnlineViewer;
import it.units.erallab.hmsrobots.viewers.drawers.Drawers;
import org.apache.commons.lang3.tuple.Pair;
import org.dyn4j.dynamics.Settings;

import java.util.Random;
import java.util.stream.IntStream;

/**
 * @author Eric Medvet <eric.medvet@gmail.com>
 */
public class Starter {

  public static void main(String[] args) {
    randomNonUniformDirectionalMLP();
  }

  private static void randomNonUniformDirectionalMLP() {
    // body creation
    Grid<? extends SensingVoxel> bipedBody = RobotUtils.buildSensorizingFunction("uniform-t+a+vxy-0.01").apply(RobotUtils.buildShape("biped-4x3"));
    Grid<? extends SensingVoxel> wormBody = RobotUtils.buildSensorizingFunction("uniform-t+a+vxy-0.01").apply(RobotUtils.buildShape("worm-5x1"));
    Grid<? extends SensingVoxel> combBody = RobotUtils.buildSensorizingFunction("uniform-t+a+vxy-0.01").apply(RobotUtils.buildShape("comb-7x2"));

    // controller creation
    DistributedSensing bipedDistributedSensing = new DistributedSensing(bipedBody, 1);
    DistributedSensing wormDistributedSensing = new DistributedSensing(wormBody, 1);
    DistributedSensing combDistributedSensing = new DistributedSensing(combBody, 1);
    fillWithRandomMLPs(bipedBody, bipedDistributedSensing);
    fillWithRandomMLPs(wormBody, wormDistributedSensing);
    fillWithRandomMLPs(combBody, combDistributedSensing);

    // robot creation
    Robot<SensingVoxel> biped = new Robot<>(bipedDistributedSensing, SerializationUtils.clone(bipedBody));
    Robot<SensingVoxel> worm = new Robot<>(wormDistributedSensing, SerializationUtils.clone(wormBody));
    Robot<SensingVoxel> comb = new Robot<>(combDistributedSensing, SerializationUtils.clone(combBody));

    // locomotion task
    Locomotion locomotion = new Locomotion(
        30,
        Locomotion.createTerrain("downhill-30"),
        new Settings()
    );

    Grid<Pair<String, Robot<?>>> namedSolutionGrid = Grid.create(3, 1);
    namedSolutionGrid.set(0, 0, Pair.of("biped", biped));
    namedSolutionGrid.set(1, 0, Pair.of("worm", worm));
    namedSolutionGrid.set(2, 0, Pair.of("comb", comb));
    GridOnlineViewer.run(locomotion, namedSolutionGrid, Drawers::basicWithMiniWorld);
  }

  private static void fillWithRandomMLPs(Grid<? extends SensingVoxel> body, DistributedSensing distributedSensing) {
    Random random = new Random();
    for (Grid.Entry<? extends SensingVoxel> entry : body) {
      MultiLayerPerceptron mlp = new MultiLayerPerceptron(
          MultiLayerPerceptron.ActivationFunction.TANH,
          distributedSensing.nOfInputs(entry.getX(), entry.getY()),
          new int[]{2},
          distributedSensing.nOfOutputs(entry.getX(), entry.getY())
      );
      double[] ws = mlp.getParams();
      IntStream.range(0, ws.length).forEach(i -> ws[i] = random.nextDouble() * 2d - 1d);
      mlp.setParams(ws);
      distributedSensing.getFunctions().set(entry.getX(), entry.getY(), mlp);
    }
  }


}
