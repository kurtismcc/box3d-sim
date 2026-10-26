# Step 01: Project scaffold

**Goal:** an empty, buildable Java 25 library in `F:\Projects\box3d-sim` that publishes to Maven Local.

**Depends on:** nothing.

## Decisions (already made)

- Single Gradle project. No subprojects.
- Maven coordinates `box3dsim:box3d-sim:0.1.0-SNAPSHOT`. Java package root `box3dsim`. Both are easy to rename later.
- The library does **not** depend on WPILib. Its own small math types are converted to WPILib types in the robot
  repo (step 11). This keeps the library off the 2027 alpha artifacts and lets unit tests run as plain JUnit.
- Java toolchain 25, matching the robot (`sourceCompatibility = VERSION_25`).
- Gradle wrapper 9.6.1, copied from the robot repo.

## Files

### Gradle wrapper

Copy these from `D:\Projects\2026-Offseason` unchanged:

- `gradlew`
- `gradlew.bat`
- `gradle/wrapper/gradle-wrapper.jar`
- `gradle/wrapper/gradle-wrapper.properties`

Then edit `gradle-wrapper.properties`: change `distributionPath=permwrapper/dists` to `distributionPath=wrapper/dists`
and `zipStorePath=permwrapper/dists` to `zipStorePath=wrapper/dists`. `permwrapper` is a GradleRIO convention; this is
not a GradleRIO project. Keep `distributionUrl` (`gradle-9.6.1-bin.zip`).

### `settings.gradle`

```groovy
plugins {
    // Lets Gradle download JDK 25 if no local JDK 25 is found.
    id 'org.gradle.toolchains.foojay-resolver-convention' version '1.0.0'
}

rootProject.name = 'box3d-sim'
```

### `build.gradle`

```groovy
plugins {
    id 'java-library'
    id 'maven-publish'
}

group = 'box3dsim'
version = '0.1.0-SNAPSHOT'

java {
    toolchain {
        languageVersion = JavaLanguageVersion.of(25)
    }
    withSourcesJar()
}

repositories {
    mavenCentral()
}

dependencies {
    testImplementation platform('org.junit:junit-bom:5.11.4')
    testImplementation 'org.junit.jupiter:junit-jupiter'
    testRuntimeOnly 'org.junit.platform:junit-platform-launcher'
}

tasks.withType(JavaCompile).configureEach {
    options.encoding = 'UTF-8'
    options.compilerArgs.addAll(['-Xlint:all', '-Xlint:-serial'])
}

test {
    useJUnitPlatform()
    jvmArgs '--enable-native-access=ALL-UNNAMED'
    testLogging {
        events 'failed'
        exceptionFormat = 'full'
    }
}

publishing {
    publications {
        maven(MavenPublication) {
            from components.java
        }
    }
}
```

If JDK 25 is installed locally (for example after the WPILib 2027 installer puts one in
`C:\Users\Public\wpilib\2027_alpha7\jdk`), Gradle finds it. If not, the foojay plugin downloads one. If the foojay
plugin version `1.0.0` doesn't resolve, use the newest `0.x`/`1.x` listed at
https://plugins.gradle.org/plugin/org.gradle.toolchains.foojay-resolver-convention.

### `.gitignore`

```
.gradle/
build/
native-build/
tools/
logs/
*.wpilog
.idea/
*.iml
.vscode/
```

### `.gitattributes`

```
*.dll binary
*.so binary
*.dylib binary
gradlew text eol=lf
*.bat text eol=crlf
```

### Package skeleton

Create these directories. Later steps fill them; for now each holds only `package-info.java` with a one-line Javadoc.

| Package | Purpose (one-line Javadoc text) |
|---|---|
| `box3dsim.math` | Vector, quaternion and pose types used across the library. |
| `box3dsim.ffm` | Native library loading and Box3D FFM bindings. |
| `box3dsim.world` | Backend-neutral physics world API and the Box3D backend. |
| `box3dsim.motor` | DC motor, battery and lumped mechanism models. |
| `box3dsim.drive` | Swerve drivetrain and gyro models. |
| `box3dsim.gamepiece` | Game piece bodies, pooling and aerodynamics. |
| `box3dsim.mechanism` | Intake, inventory and shooter models that move game pieces. |
| `box3dsim.arena` | Top-level simulation loop tying world, robots and season together. |
| `box3dsim.season.rebuilt2026` | 2026 REBUILT field geometry, game pieces and match rules. |

All under `src/main/java/`. Tests mirror the same packages under `src/test/java/`.

### `src/test/java/box3dsim/ScaffoldTest.java`

One JUnit 5 test asserting `Runtime.version().feature() >= 25`. It proves the toolchain is wired up.

### README

In `README.md`, change the `## Status` section to:

```
## Status

v0 in progress. See [docs/plan.md](docs/plan.md).
```

## Acceptance

Run from `F:\Projects\box3d-sim`:

```
.\gradlew.bat build
.\gradlew.bat publishToMavenLocal
```

- Both succeed.
- `%USERPROFILE%\.m2\repository\box3dsim\box3d-sim\0.1.0-SNAPSHOT\` contains `box3d-sim-0.1.0-SNAPSHOT.jar`.
- `ScaffoldTest` passed (`build/reports/tests/test/index.html`).

Commit: `Scaffold Gradle library project`.
