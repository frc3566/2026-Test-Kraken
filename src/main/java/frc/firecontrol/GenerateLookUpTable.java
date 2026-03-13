package frc.firecontrol;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Locale;

public class GenerateLookUpTable {

	private static final Path DEFAULT_OUTPUT = Paths.get("build", "generated", "firecontrol", "shooter_lut.csv");

	private GenerateLookUpTable() {}

	public static void main(String[] args) throws IOException {
		Locale.setDefault(Locale.US);

		Path outputPath = args.length > 0 ? Paths.get(args[0]) : DEFAULT_OUTPUT;

		ProjectileSimulator.SimParameters params = new ProjectileSimulator.SimParameters(
			0.215,
			0.1501,
			0.47,
			0.2,
			1.225,
			0.43,
			0.1016,
			1.83,
			0.6,
			45.0,
			0.001,
			1500,
			6000,
			25,
			5.0
		);

		writeCsv(outputPath, params);
	}

	public static void writeCsv(Path outputPath, ProjectileSimulator.SimParameters params) throws IOException {
		ProjectileSimulator simulator = new ProjectileSimulator(params);
		ProjectileSimulator.GeneratedLUT lut = simulator.generateLUT();

		Files.createDirectories(outputPath.toAbsolutePath().getParent());

		StringBuilder csv = new StringBuilder();
		csv.append("distance_m,rpm,tof_s,reachable,exit_velocity_mps,z_at_target_m,max_height_m,apex_x_m\n");

		for (ProjectileSimulator.LUTEntry entry : lut.entries()) {
			if (entry.reachable()) {
				ProjectileSimulator.TrajectoryResult result = simulator.simulate(entry.rpm(), entry.distanceM());
				csv.append(String.format(
					Locale.US,
					"%.2f,%.1f,%.4f,true,%.4f,%.4f,%.4f,%.4f%n",
					entry.distanceM(),
					entry.rpm(),
					entry.tof(),
					simulator.exitVelocity(entry.rpm()),
					result.zAtTarget(),
					result.maxHeight(),
					result.apexX()));
			} else {
				csv.append(String.format(
					Locale.US,
					"%.2f,,,false,,,,%n",
					entry.distanceM()));
			}
		}

		Files.writeString(outputPath, csv.toString(), StandardCharsets.UTF_8);

		System.out.printf("Generated %d LUT entries (%d reachable, %d unreachable)%n",
			lut.entries().size(), lut.reachableCount(), lut.unreachableCount());
		System.out.printf("Max reachable range: %.2f m%n", lut.maxRangeM());
		System.out.printf("Generation time: %d ms%n", lut.generationTimeMs());
		System.out.printf("Wrote CSV to %s%n", outputPath.toAbsolutePath());
	}
}
