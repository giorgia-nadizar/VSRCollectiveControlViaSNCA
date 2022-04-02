package it.units.erallab.evolution.builder.robot;

import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;
import it.units.erallab.hmsrobots.core.controllers.DistributedSensing;
import it.units.erallab.hmsrobots.core.controllers.RealFunction;
import it.units.erallab.hmsrobots.core.controllers.TimedRealFunction;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.core.sensors.Constant;
import it.units.erallab.hmsrobots.core.sensors.Sensor;
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
public class SensorAndBodyAndHomoDistributed implements PrototypedFunctionBuilder<List<TimedRealFunction>, Robot<? extends SensingVoxel>> {
  private final int signals;
  private final double percentile;
  private final boolean withPositionSensors;

  public SensorAndBodyAndHomoDistributed(int signals, double percentile, boolean withPositionSensors) {
    this.signals = signals;
    this.percentile = percentile;
    this.withPositionSensors = withPositionSensors;
  }

  @Override
  public Function<List<TimedRealFunction>, Robot<? extends SensingVoxel>> buildFor(Robot<? extends SensingVoxel> robot) {
    int w = robot.getVoxels().getW();
    int h = robot.getVoxels().getH();
    List<Sensor> prototypeSensors = getPrototypeSensors(robot);
    int nOfInputs = DistributedSensing.nOfInputs(new SensingVoxel(prototypeSensors.subList(0, 1)), signals) + (withPositionSensors ? 2 : 0);
    int nOfOutputs = DistributedSensing.nOfOutputs(new SensingVoxel(prototypeSensors.subList(0, 1)), signals);
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
      if (bodyFunction.getInputDimension() != 2 || bodyFunction.getOutputDimension() != prototypeSensors.size()) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of body function args: 2->%d expected, %d->%d found",
            prototypeSensors.size(),
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
      Grid<double[]> values = Grid.create(
          w, h,
          (x, y) -> bodyFunction.apply(0d, new double[]{(double) x / ((double) w - 1d), (double) y / ((double) h - 1d)})
      );
      double threshold = new Percentile().evaluate(
          values.values().stream().mapToDouble(SensorAndBodyAndHomoDistributed::max).toArray(),
          percentile
      );
      values = Grid.create(values, vs -> max(vs) >= threshold ? vs : null);
      values = Utils.gridLargestConnected(values, Objects::nonNull);
      values = Utils.cropGrid(values, Objects::nonNull);
      Grid<SensingVoxel> body = new Grid<>(values.getW(), values.getH(), null);
      for (int x = 0; x < body.getW(); x++) {
        for (int y = 0; y < body.getH(); y++) {
          int rx = (int) Math.floor((double) x / (double) body.getW() * (double) w);
          int ry = (int) Math.floor((double) y / (double) body.getH() * (double) h);
          if (values.get(x, y) != null) {
            List<Sensor> availableSensors = robot.getVoxels().get(rx, ry) != null ? robot.getVoxels().get(rx, ry).getSensors() : prototypeSensors;
            body.set(x, y, new SensingVoxel(withPositionSensors ?
                List.of(
                    SerializationUtils.clone(availableSensors.get(indexOfMax(values.get(x, y)))),
                    new Constant((double) x / ((double) body.getW() - 1d), (double) y / ((double) body.getH() - 1d))
                ) :
                List.of(
                    SerializationUtils.clone(availableSensors.get(indexOfMax(values.get(x, y))))
                )
            ));
          }
        }
      }
      if (body.values().stream().noneMatch(Objects::nonNull)) {
        body = Grid.create(1, 1, new SensingVoxel(List.of(SerializationUtils.clone(prototypeSensors.get(indexOfMax(values.get(0, 0)))))));
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

  static double max(double[] vs) {
    double max = vs[0];
    for (int i = 1; i < vs.length; i++) {
      max = Math.max(max, vs[i]);
    }
    return max;
  }

  static int indexOfMax(double[] vs) {
    int indexOfMax = 0;
    for (int i = 1; i < vs.length; i++) {
      if (vs[i] > vs[indexOfMax]) {
        indexOfMax = i;
      }
    }
    return indexOfMax;
  }

  @Override
  public List<TimedRealFunction> exampleFor(Robot<? extends SensingVoxel> robot) {
    List<Sensor> sensors = getPrototypeSensors(robot);
    return List.of(
        RealFunction.build(d -> d, 2, sensors.size()),
        RealFunction.build(
            d -> d,
            DistributedSensing.nOfInputs(new SensingVoxel(sensors.subList(0, 1)), signals) + (withPositionSensors ? 2 : 0),
            DistributedSensing.nOfOutputs(new SensingVoxel(sensors.subList(0, 1)), signals)
        )
    );
  }

  static List<Sensor> getPrototypeSensors(Robot<? extends SensingVoxel> robot) {
    SensingVoxel voxelPrototype = robot.getVoxels().values().stream().filter(Objects::nonNull).findFirst().orElse(null);
    if (voxelPrototype == null) {
      throw new IllegalArgumentException("Target robot has no voxels");
    }
    if (voxelPrototype.getSensors().isEmpty()) {
      throw new IllegalArgumentException("Target robot has no sensors");
    }
    if (voxelPrototype.getSensors().stream().mapToInt(s -> s.getDomains().length).distinct().count() != 1) {
      throw new IllegalArgumentException(String.format(
          "Target robot has sensors with different number of outputs: %s",
          voxelPrototype.getSensors().stream().mapToInt(s -> s.getDomains().length).distinct()
      ));
    }
    return voxelPrototype.getSensors();
  }

}
