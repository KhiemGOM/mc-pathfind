plugins {
    application
}

dependencies {
    implementation("it.unimi.dsi:fastutil:8.5.18")
}

application {
    mainClass.set("dev.mcpathfind.pearl.bench.PearlBenchmarkCli")
}
