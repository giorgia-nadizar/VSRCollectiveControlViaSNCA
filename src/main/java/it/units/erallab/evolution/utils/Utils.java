/*
 * Copyright 2020 Eric Medvet <eric.medvet@gmail.com> (as eric)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package it.units.erallab.evolution.utils;

import it.units.erallab.evolution.LocomotionEvolution;
import it.units.erallab.hmsrobots.behavior.BehaviorUtils;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.snapshots.VoxelPoly;
import it.units.erallab.hmsrobots.tasks.locomotion.Locomotion;
import it.units.erallab.hmsrobots.tasks.locomotion.Outcome;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.RobotUtils;
import it.units.erallab.hmsrobots.util.SerializationUtils;
import it.units.erallab.hmsrobots.viewers.GridFileWriter;
import it.units.erallab.hmsrobots.viewers.VideoUtils;
import it.units.erallab.hmsrobots.viewers.drawers.Drawer;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.Event;
import it.units.malelab.jgea.core.listener.Accumulator;
import it.units.malelab.jgea.core.listener.NamedFunction;
import it.units.malelab.jgea.core.listener.NamedFunctions;
import it.units.malelab.jgea.core.listener.TableBuilder;
import it.units.malelab.jgea.core.util.*;
import org.dyn4j.dynamics.Settings;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.*;
import java.util.function.Function;
import java.util.logging.Logger;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static it.units.malelab.jgea.core.listener.NamedFunctions.*;

/**
 * @author eric
 */
public class Utils {

  private static final Logger L = Logger.getLogger(Utils.class.getName());

  private Utils() {
  }

  public static List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> keysFunctions() {
    return List.of(
        eventAttribute("experiment.name"),
        eventAttribute("seed", "%2d"),
        eventAttribute("terrain"),
        eventAttribute("shape"),
        eventAttribute("sensor.config"),
        eventAttribute("mapper"),
        eventAttribute("transformation"),
        eventAttribute("evolver"),
        eventAttribute("fitness.metrics")
    );
  }

  public static List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> basicFunctions() {
    NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?> elapsedSeconds = elapsedSeconds();
    return List.of(
        iterations(),
        births(),
        fitnessEvaluations(),
        elapsedSeconds.reformat("%6.0f")
    );
  }

  public static List<NamedFunction<Individual<?, ? extends Robot<?>, ? extends Outcome>, ?>> serializationFunction(boolean flag, NamedFunction<Individual<?, ?, ?>, ?> function) {
    if (!flag) {
      return List.of();
    }
    return List.of(f("serialized", r -> SerializationUtils.serialize(r, SerializationUtils.Mode.GZIPPED_JSON)).of(function));
  }

  public static List<NamedFunction<Individual<?, ? extends Robot<?>, ? extends Outcome>, ?>> individualFunctions(Function<Outcome, Double> fitnessFunction) {
    NamedFunction<Individual<?, ? extends Robot<?>, ? extends Outcome>, ?> size = size().of(genotype());
    return List.of(
        f("w", "%2d", (Function<Grid<?>, Number>) Grid::getW)
            .of(f("shape", (Function<Robot<?>, Grid<?>>) Robot::getVoxels))
            .of(solution()),
        f("h", "%2d", (Function<Grid<?>, Number>) Grid::getH)
            .of(f("shape", (Function<Robot<?>, Grid<?>>) Robot::getVoxels))
            .of(solution()),
        f("num.voxel", "%2d", (Function<Grid<?>, Number>) g -> g.count(Objects::nonNull))
            .of(f("shape", (Function<Robot<?>, Grid<?>>) Robot::getVoxels))
            .of(solution()),
        size.reformat("%5d"),
        genotypeBirthIteration(),
        f("fitness", "%5.1f", fitnessFunction).of(fitness()),
        f("abs.weights.sum", "%5.1f", Outcome::getInitialSumOfAbsoluteWeights).of(fitness()),
        f("avg.abs.weights.sum", "%5.1f", Outcome::getAverageSumOfAbsoluteWeights).of(fitness())
    );
  }

  public static List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> populationFunctions(Function<Outcome, Double> fitnessFunction) {
    NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?> min = min(Double::compare).of(each(f("fitness", fitnessFunction).of(fitness()))).of(all());
    NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?> median = median(Double::compare).of(each(f("fitness", fitnessFunction).of(fitness()))).of(all());
    return List.of(
        size().of(all()),
        size().of(firsts()),
        size().of(lasts()),
        uniqueness().of(each(genotype())).of(all()),
        uniqueness().of(each(solution())).of(all()),
        uniqueness().of(each(fitness())).of(all()),
        min.reformat("%+4.1f"),
        median.reformat("%5.1f")
    );
  }

  public static List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> computationTimeFunctions() {
    Function<Outcome, Double> computationTimeFunction = Outcome::getComputationTime;
    NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?> min = min(Double::compare).of(each(f("computation.time", computationTimeFunction).of(fitness()))).of(all());
    NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?> max = max(Double::compare).of(each(f("computation.time", computationTimeFunction).of(fitness()))).of(all());
    return List.of(
        min.reformat("%4.1f"),
        max.reformat("%5.1f")
    );
  }

  public static List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> visualFunctions(Function<Outcome, Double> fitnessFunction) {
    return List.of(
        hist(8)
            .of(each(f("fitness", fitnessFunction).of(fitness())))
            .of(all()),
        hist(8)
            .of(each(f("num.voxels", (Function<Grid<?>, Number>) g -> g.count(Objects::nonNull))
                .of(f("shape", (Function<Robot<?>, Grid<?>>) Robot::getVoxels))
                .of(solution())))
            .of(all()),
        f("minimap", "%4s", (Function<Grid<?>, String>) g -> TextPlotter.binaryMap(
            g.toArray(Objects::nonNull),
            (int) Math.min(Math.ceil((float) g.getW() / (float) g.getH() * 2f), 4)))
            .of(f("shape", (Function<Robot<?>, Grid<?>>) Robot::getVoxels))
            .of(solution()).of(best()),
        f("average.posture.minimap", "%2s", (Function<Outcome, String>) o -> TextPlotter.binaryMap(o.getAveragePosture(8).toArray(b -> b), 2))
            .of(fitness()).of(best())
    );
  }

  public static List<NamedFunction<Outcome, ?>> basicOutcomeFunctions() {
    return List.of(
        f("computation.time", "%4.2f", Outcome::getComputationTime),
        f("distance", "%5.1f", Outcome::getDistance),
        f("velocity", "%5.1f", Outcome::getVelocity),
        f("corrected.efficiency", "%5.2f", Outcome::getCorrectedEfficiency),
        f("area.ratio.power", "%5.1f", Outcome::getAreaRatioPower),
        f("control.power", "%5.1f", Outcome::getControlPower)
    );
  }

  public static List<NamedFunction<Outcome, ?>> detailedOutcomeFunctions(double spectrumMinFreq, double spectrumMaxFreq, int spectrumSize) {
    return Misc.concat(List.of(
        NamedFunction.then(cachedF(
                "center.x.spectrum",
                (Outcome o) -> new ArrayList<>(o.getCenterXVelocitySpectrum(spectrumMinFreq, spectrumMaxFreq, spectrumSize).values())
            ),
            IntStream.range(0, spectrumSize).mapToObj(NamedFunctions::nth).collect(Collectors.toList())
        ),
        NamedFunction.then(cachedF(
                "center.y.spectrum",
                (Outcome o) -> new ArrayList<>(o.getCenterYVelocitySpectrum(spectrumMinFreq, spectrumMaxFreq, spectrumSize).values())
            ),
            IntStream.range(0, spectrumSize).mapToObj(NamedFunctions::nth).collect(Collectors.toList())
        ),
        NamedFunction.then(cachedF(
                "center.angle.spectrum",
                (Outcome o) -> new ArrayList<>(o.getCenterAngleSpectrum(spectrumMinFreq, spectrumMaxFreq, spectrumSize).values())
            ),
            IntStream.range(0, spectrumSize).mapToObj(NamedFunctions::nth).collect(Collectors.toList())
        ),
        NamedFunction.then(cachedF(
                "footprints.spectra",
                (Outcome o) -> o.getFootprintsSpectra(4, spectrumMinFreq, spectrumMaxFreq, spectrumSize).stream()
                    .map(SortedMap::values)
                    .flatMap(Collection::stream)
                    .collect(Collectors.toList())
            ),
            IntStream.range(0, 4 * spectrumSize).mapToObj(NamedFunctions::nth).collect(Collectors.toList())
        )
    ));
  }

  public static List<NamedFunction<Outcome, ?>> visualOutcomeFunctions(double spectrumMinFreq, double spectrumMaxFreq) {
    return Misc.concat(List.of(
        List.of(
            cachedF("center.x.spectrum", "%4.4s", o -> TextPlotter.barplot(
                new ArrayList<>(o.getCenterXVelocitySpectrum(spectrumMinFreq, spectrumMaxFreq, 4).values())
            )),
            cachedF("center.y.spectrum", "%4.4s", o -> TextPlotter.barplot(
                new ArrayList<>(o.getCenterYVelocitySpectrum(spectrumMinFreq, spectrumMaxFreq, 4).values())
            )),
            cachedF("center.angle.spectrum", "%4.4s", o -> TextPlotter.barplot(
                new ArrayList<>(o.getCenterAngleSpectrum(spectrumMinFreq, spectrumMaxFreq, 4).values())
            ))
        ),
        NamedFunction.then(cachedF("footprints", o -> o.getFootprintsSpectra(3, spectrumMinFreq, spectrumMaxFreq, 4)),
            List.of(
                cachedF("left.spectrum", "%4.4s", l -> TextPlotter.barplot(new ArrayList<>(l.get(0).values()))),
                cachedF("center.spectrum", "%4.4s", l -> TextPlotter.barplot(new ArrayList<>(l.get(1).values()))),
                cachedF("right.spectrum", "%4.4s", l -> TextPlotter.barplot(new ArrayList<>(l.get(2).values())))
            )
        )
    ));
  }

  public static Accumulator.Factory<Event<?, ? extends Robot<?>, ? extends Outcome>, String> lastEventToString(Function<Outcome, Double> fitnessFunction) {
    final List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> functions = Misc.concat(List.of(
        keysFunctions(),
        basicFunctions(),
        populationFunctions(fitnessFunction),
        NamedFunction.then(best(), individualFunctions(fitnessFunction)),
        NamedFunction.then(as(Outcome.class).of(fitness()).of(best()), basicOutcomeFunctions())
    ));
    return Accumulator.Factory.<Event<?, ? extends Robot<?>, ? extends Outcome>>last().then(
        e -> functions.stream()
            .map(f -> f.getName() + ": " + f.applyAndFormat(e))
            .collect(Collectors.joining("\n"))
    );
  }

  public static Accumulator.Factory<Event<?, ? extends Robot<?>, ? extends Outcome>, BufferedImage> fitnessPlot(Function<Outcome, Double> fitnessFunction) {
    return new TableBuilder<Event<?, ? extends Robot<?>, ? extends Outcome>, Number>(List.of(
        iterations(),
        f("fitness", fitnessFunction).of(fitness()).of(best()),
        min(Double::compare).of(each(f("fitness", fitnessFunction).of(fitness()))).of(all()),
        median(Double::compare).of(each(f("fitness", fitnessFunction).of(fitness()))).of(all())
    )).then(ImagePlotters.xyLines(600, 400));
  }

  public static Accumulator.Factory<Event<?, ? extends Robot<?>, ? extends Outcome>, BufferedImage> centerPositionPlot() {
    return Accumulator.Factory.<Event<?, ? extends Robot<?>, ? extends Outcome>>last().then(
            event -> {
              Outcome o = Misc.first(event.getOrderedPopulation().firsts()).getFitness();
              Table<Number> table = new ArrayTable<>(List.of("x", "y", "terrain.y"));
              o.getObservations().values().forEach(obs -> {
                VoxelPoly poly = BehaviorUtils.getCentralElement(obs.getVoxelPolies());
                table.addRow(List.of(
                    poly.center().x,
                    poly.center().y,
                    obs.getTerrainHeight()
                ));
              });
              return table;
            }
        )
        .then(ImagePlotters.xyLines(600, 400));
  }

  public static Accumulator.Factory<Event<?, ? extends Robot<?>, ? extends Outcome>, File> bestVideo(double transientTime, double episodeTime, Settings settings, Pair<Pair<Integer, Integer>, Function<String, Drawer>> drawerSupplier) {
    int widthMultiplier = drawerSupplier.first().first();
    int heightMultiplier = drawerSupplier.first().second();
    return Accumulator.Factory.<Event<?, ? extends Robot<?>, ? extends Outcome>>last().then(
        event -> {
          Random random = new Random(0);
          SortedMap<Long, String> terrainSequence = LocomotionEvolution.getSequence((String) event.getAttributes().get("terrain"));
          SortedMap<Long, String> transformationSequence = LocomotionEvolution.getSequence((String) event.getAttributes().get("transformation"));
          String terrainName = terrainSequence.get(terrainSequence.lastKey());
          String transformationName = transformationSequence.get(transformationSequence.lastKey());
          Robot<?> robot = SerializationUtils.clone(Misc.first(event.getOrderedPopulation().firsts()).getSolution());
          robot = RobotUtils.buildRobotTransformation(transformationName, random).apply(robot);
          Locomotion locomotion = new Locomotion(
              episodeTime,
              Locomotion.createTerrain(terrainName.replace("-rnd", "-" + random.nextInt(10000))),
              settings
          );
          File file;
          try {
            file = File.createTempFile("robot-video", ".mp4");
            GridFileWriter.save(locomotion, robot, widthMultiplier * 300, heightMultiplier * 200, transientTime, 25, VideoUtils.EncoderFacility.JCODEC, file, drawerSupplier.second());
            file.deleteOnExit();
          } catch (IOException ioException) {
            L.warning(String.format("Cannot save video of best: %s", ioException));
            return null;
          }
          return file;
        }
    );
  }

  public static Function<Event<?, ? extends Robot<?>, ? extends Outcome>, Collection<LocomotionEvolution.ValidationOutcome>> validation(
      List<String> validationTerrainNames,
      List<String> validationTransformationNames,
      List<Integer> seeds,
      double episodeTime
  ) {
    return event -> {
      Robot<?> robot = SerializationUtils.clone(Misc.first(event.getOrderedPopulation().firsts()).getSolution());
      List<LocomotionEvolution.ValidationOutcome> validationOutcomes = new ArrayList<>();
      for (String validationTerrainName : validationTerrainNames) {
        for (String validationTransformationName : validationTransformationNames) {
          for (int seed : seeds) {
            Random random = new Random(seed);
            robot = SerializationUtils.clone(robot, SerializationUtils.Mode.GZIPPED_JSON);
            robot = RobotUtils.buildRobotTransformation(validationTransformationName, random).apply(robot);
            Function<Robot<?>, Outcome> validationLocomotion = LocomotionEvolution.buildLocomotionTask(
                validationTerrainName,
                episodeTime,
                random,
                false
            );
            Outcome outcome = validationLocomotion.apply(robot);
            validationOutcomes.add(new LocomotionEvolution.ValidationOutcome(
                event,
                Map.ofEntries(
                    Map.entry("validation.terrain", validationTerrainName),
                    Map.entry("validation.transformation", validationTransformationName),
                    Map.entry("validation.seed", seed)
                ),
                outcome
            ));
          }
        }
      }
      return validationOutcomes;
    };
  }
}
