plugins {
    `java-library`
}

dependencies {
    implementation("it.unimi.dsi:fastutil:8.5.18")

    testImplementation("org.junit.jupiter:junit-jupiter:5.10.2")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}

tasks.test {
    useJUnitPlatform()
}
