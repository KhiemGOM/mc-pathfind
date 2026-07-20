package dev.mcpathfind.pearl.io;

import dev.mcpathfind.pearl.World;

import java.nio.file.Path;
import java.security.MessageDigest;

/** Throwaway smoke test: verifies WorldBinFormat.load() against an independently-computed Python checksum. */
public final class RoundTripCheck {
    public static void main(String[] args) throws Exception {
        WorldBinFormat.Loaded loaded = WorldBinFormat.load(Path.of(args[0]));
        World w = loaded.world();
        System.out.println("shape=" + w.sizeX + "x" + w.sizeY + "x" + w.sizeZ);
        System.out.println("origin=" + loaded.origin());
        System.out.println("start=" + loaded.start());
        System.out.println("goal=" + loaded.goal());
        System.out.println("blocksAvailable=" + loaded.blocksAvailable());

        long sum = 0;
        long nonzero = 0;
        for (byte b : w.blocks) {
            sum += b;
            if (b != 0) nonzero++;
        }
        System.out.println("sum=" + sum + " nonzero=" + nonzero);

        MessageDigest sha256 = MessageDigest.getInstance("SHA-256");
        byte[] digest = sha256.digest(w.blocks);
        StringBuilder hex = new StringBuilder();
        for (byte b : digest) hex.append(String.format("%02x", b));
        System.out.println("sha256=" + hex);
    }
}
