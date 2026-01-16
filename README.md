# GRRBase

A template repository for Java-based robots in the FIRST Robotics Competition.

## Highlights

- [Swerve API](src/main/java/org/team340/lib/swerve/SwerveAPI.java)

    Supports various hardware configurations, from vendors such as CTRE, REV, and Redux Robotics, with features such as high frequency odometry, a custom ratelimiter to improve driver control while also reducing wheel scrub, and built-in support for tuning the drivetrain's configuration live via NetworkTables.

- [Pre-defined Loggers](src/main/java/org/team340/lib/logging)

    Utilizing Epilogue, an annotation-based logging framework, custom loggers for vendor and WPILib APIs are pre-defined to transparently log relevant data from the robot's hardware.

- [Various other utilities](src/main/java/org/team340/lib/util)

## Getting Started

### Creating your repository

1. On GitHub, navigate to the main page of this repository.
2. Above the file list, click *Use this template*.
3. Select *Create a new repository*.

*See also: [GitHub Docs](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template)*

### Prerequisites

- [WPILib](https://github.com/wpilibsuite/allwpilib/releases) 2026.x
- [Node.js](https://nodejs.org/en/download) (v24 LTS Recommended)

Node.js is required to support linting via [Spotless](https://github.com/diffplug/spotless), using the [Prettier](https://prettier.io) style guide. You can apply these rules by building, or by running `./gradlew spotlessApply`. Alternatively, if you wish to disable code formatting, you can do so with the following modifications to [build.gradle](build.gradle):

<details>

<summary>Expand</summary>

<br>

```diff
diff --git a/build.gradle b/build.gradle
--- a/build.gradle
+++ b/build.gradle
@@ -1,7 +1,6 @@
 plugins {
     id "java"
     id "edu.wpi.first.GradleRIO" version "2026.2.1"
-    id "com.diffplug.spotless" version "8.1.0"
 }

 java {
@@ -92,32 +91,6 @@ dependencies {
     testRuntimeOnly 'org.junit.platform:junit-platform-launcher'
 }

-// Code formatting via spotless
-spotless {
-    java {
-        target fileTree('.') {
-            include '**/*.java'
-            exclude '**/build/**', '**/build-*/**'
-        }
-
-        toggleOffOn()
-        endWithNewline()
-        removeUnusedImports()
-        trimTrailingWhitespace()
-        prettier(['prettier': '3.8.0', 'prettier-plugin-java': '2.8.1'])
-            .config([
-                parser: 'java',
-                plugins: ['prettier-plugin-java'],
-                arrowParens: 'avoid',
-                experimentalOperatorPosition: 'start',
-                printWidth: 120,
-                tabWidth: 4,
-                useTabs: false,
-                trailingComma: 'none'
-            ])
-    }
-}
-
 test {
     useJUnitPlatform()
     systemProperty 'junit.jupiter.extensions.autodetection.enabled', 'true'
@@ -147,5 +120,4 @@ wpi.java.configureTestTasks(test)
 // Configure string concat to always inline compile
 tasks.withType(JavaCompile) {
     options.compilerArgs.add '-XDstringConcat=inline'
-    dependsOn 'spotlessApply'
 }
```

*You may also want to modify [workflows/main.yml](.github/workflows/main.yml) to remove the formatting check from CI:*

```diff
diff --git a/.github/workflows/main.yml b/.github/workflows/main.yml
--- a/.github/workflows/main.yml
+++ b/.github/workflows/main.yml
@@ -5,32 +5,6 @@ env:
   NODE_VERSION: 24

 jobs:
-  format:
-    name: Check Format
-    runs-on: ubuntu-latest
-    container: wpilib/ubuntu-base:22.04
-
-    steps:
-      - name: Checkout Repository
-        uses: actions/checkout@v6
-        with:
-          fetch-depth: 0
-
-      - name: Add repository to git safe directories
-        run: git config --global --add safe.directory $GITHUB_WORKSPACE
-
-      - name: Setup Node
-        uses: actions/setup-node@v6
-        with:
-          node-version: ${{ env.NODE_VERSION }}
-          registry-url: https://registry.npmjs.org/
-
-      - name: Grant execute permission for gradlew
-        run: chmod +x gradlew
-
-      - name: Check robot code formatting
-        run: ./gradlew spotlessCheck
-
   build:
     name: Build
     runs-on: ubuntu-latest
@@ -54,8 +28,5 @@ jobs:
       - name: Grant execute permission for gradlew
         run: chmod +x gradlew

-      - name: Ensure spotless will not fail on build
-        run: ./gradlew spotlessApply
-
       - name: Compile and run tests on robot code
         run: ./gradlew build
```

</details>

### Configuring the project for your team

First, ensure [wpilib_preferences.json](.wpilib/wpilib_preferences.json) is configured for your team number:

```json
{
    "teamNumber": 340
}
```

*(Optional)* If you decide to modify the package name for your robot code (by default, `org.team340.robot`), you must update your [build.gradle](build.gradle) to reflect the new location of the project's `Main` class:

```groovy
def ROBOT_MAIN_CLASS = "com.example.robot.Main"
```

## Versioning

Due to the nature of GitHub templates, GRRBase is intentionally not versioned. It may be helpful to save the commit hash utilized when creating your repository for future reference. The `main` branch of this repository should be expected to always remain stable.

If you wish to manually sync changes to GRRBase with your project, simply copying the `org.team340.lib` package should suffice, as the `robot` package is only provided as example code. You should always independently check for dependency updates (see [build.gradle](build.gradle) and [vendordeps](vendordeps)), as it is likely for GRRBase to occasionally fall out of date.
