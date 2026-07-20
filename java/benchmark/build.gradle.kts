plugins {
    application
}

dependencies {
    implementation(project(":core"))
}

application {
    mainClass.set("dev.mcpathfind.bench.BenchmarkCli")
}
