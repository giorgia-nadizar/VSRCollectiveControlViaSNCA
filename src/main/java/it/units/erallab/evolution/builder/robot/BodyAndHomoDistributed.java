package it.units.erallab.evolution.builder.robot;

import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;
import it.units.erallab.hmsrobots.core.controllers.DistributedSensing;
import it.units.erallab.hmsrobots.core.controllers.RealFunction;
import it.units.erallab.hmsrobots.core.controllers.TimedRealFunction;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.SerializationUtils;
import it.units.erallab.hmsrobots.util.Utils;
import org.apache.commons.math3.stat.descriptive.rank.Percentile;

import java.util.List;
import java.util.Objects;
import java.util.function.Function;

/**
 * @author eric
 */
public class BodyAndHomoDistributed implements PrototypedFunctionBuilder<List<TimedRealFunction>, Robot<? extends SensingVoxel>> {
  private final int signals;
  private final double percentile;

  public BodyAndHomoDistributed(int signals, double percentile) {
    this.signals = signals;
    this.percentile = percentile;
  }

  @Override
  public Function<List<TimedRealFunction>, Robot<? extends SensingVoxel>> buildFor(Robot<? extends SensingVoxel> robot) {
    int w = robot.getVoxels().getW();
    int h = robot.getVoxels().getH();
    SensingVoxel voxelPrototype = robot.getVoxels().values().stream().filter(Objects::nonNull).findFirst().orElse(null);
    if (voxelPrototype == null) {
      throw new IllegalArgumentException("Target robot has no voxels");
    }
    int nOfInputs = DistributedSensing.nOfInputs(voxelPrototype, signals);
    int nOfOutputs = DistributedSensing.nOfOutputs(voxelPrototype, signals);
    //build body
    return pair -> {
      if (pair.size() != 2) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of functions: 2 expected, %d found",
            pair.size()
        ));
      }
      TimedRealFunction bodyFunction = pair.get(0);
      TimedRealFunction brainFunction = pair.get(1);
      //check function sizes
      if (bodyFunction.getInputDimension() != 2 || bodyFunction.getOutputDimension() != 1) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of body function args: 2->1 expected, %d->%d found",
            bodyFunction.getInputDimension(),
            bodyFunction.getOutputDimension()
        ));
      }
      if (brainFunction.getInputDimension() != nOfInputs) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of brain function input args: %d expected, %d found",
            nOfInputs,
            brainFunction.getInputDimension()
        ));
      }
      if (brainFunction.getOutputDimension() != nOfOutputs) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of brain function output args: %d expected, %d found",
            nOfOutputs,
            brainFunction.getOutputDimension()
        ));
      }
      //build body
      Grid<Double> values = Grid.create(
          w, h,
          (x, y) -> bodyFunction.apply(0d, new double[]{(double) x / ((double) w - 1d), (double) y / ((double) h - 1d)})[0]
      );
      double threshold = new Percentile().evaluate(values.values().stream().mapToDouble(v -> v).toArray(), percentile);
      values = Grid.create(values, v -> v >= threshold ? v : null);
      values = Utils.gridLargestConnected(values, Objects::nonNull);
      values = Utils.cropGrid(values, Objects::nonNull);
      Grid<SensingVoxel> body = Grid.create(values, v -> (v != null) ? SerializationUtils.clone(voxelPrototype) : null);
      if (body.values().stream().noneMatch(Objects::nonNull)) {
        body = Grid.create(1, 1, SerializationUtils.clone(voxelPrototype));
      }
      //build brain
      DistributedSensing controller = new DistributedSensing(body, signals);
      for (Grid.Entry<? extends SensingVoxel> entry : body) {
        if (entry.getValue() != null) {
          controller.getFunctions().set(entry.getX(), entry.getY(), SerializationUtils.clone(brainFunction));
        }
      }
      return new Robot<>(controller, body);
    };
  }

  @Override
  public List<TimedRealFunction> exampleFor(Robot<? extends SensingVoxel> robot) {
    SensingVoxel voxelPrototype = robot.getVoxels().values().stream().filter(Objects::nonNull).findFirst().orElse(null);
    if (voxelPrototype == null) {
      throw new IllegalArgumentException("Target robot has no voxels");
    }
    return List.of(
        RealFunction.build(d -> d, 2, 1),
        RealFunction.build(
            d -> d,
            DistributedSensing.nOfInputs(voxelPrototype, signals),
            DistributedSensing.nOfOutputs(voxelPrototype, signals)
        )
    );
  }

}
