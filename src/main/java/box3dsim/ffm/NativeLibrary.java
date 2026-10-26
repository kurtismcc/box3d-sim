package box3dsim.ffm;

import box3dsim.ffm.gen.Box3D;
import box3dsim.ffm.gen.b3Version;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.lang.foreign.Arena;
import java.lang.foreign.MemorySegment;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.HexFormat;
import java.util.Locale;

/** Loads the Box3D native library bundled in the jar. */
public final class NativeLibrary {
    private static final String NATIVE_PATH_PROPERTY = "box3dsim.nativePath";

    private static Path loadedPath;
    private static String version;

    private NativeLibrary() {}

    /** Loads box3d once per JVM. Safe to call repeatedly and from any thread. */
    public static synchronized void load() {
        if (loadedPath != null) {
            return;
        }
        String override = System.getProperty(NATIVE_PATH_PROPERTY);
        Path path = override != null ? Path.of(override).toAbsolutePath() : extractBundled();
        System.load(path.toString());
        loadedPath = path;

        if (Box3D.b3IsDoublePrecision()) {
            throw new IllegalStateException(
                    "box3d was built with BOX3D_DOUBLE_PRECISION; box3d-sim needs the single-precision build");
        }
        try (Arena arena = Arena.ofConfined()) {
            MemorySegment v = Box3D.b3GetVersion(arena);
            version = b3Version.major(v) + "." + b3Version.minor(v) + "." + b3Version.revision(v);
        }
        String commit = readResource("/natives/box3d.version");
        if (commit != null) {
            version += " (" + commit.substring(0, Math.min(12, commit.length())) + ")";
        }
    }

    /** Absolute path of the loaded library file. Only valid after load(). */
    public static synchronized Path loadedPath() {
        if (loadedPath == null) {
            throw new IllegalStateException("NativeLibrary.load() has not been called");
        }
        return loadedPath;
    }

    /** Box3D version and commit, e.g. "0.1.0 (f36d061ba269)". Only valid after load(). */
    public static synchronized String version() {
        if (version == null) {
            throw new IllegalStateException("NativeLibrary.load() has not been called");
        }
        return version;
    }

    private static Path extractBundled() {
        String platform = platform();
        byte[] dll = readResourceBytes("/natives/" + platform + "/box3d.dll");
        if (dll == null) {
            throw new IllegalStateException("Missing bundled native /natives/" + platform + "/box3d.dll");
        }
        String commit = readResource("/natives/box3d.version");
        String commitTag = commit == null ? "unknown" : commit.substring(0, Math.min(12, commit.length()));
        String hash = sha256(dll).substring(0, 12);

        Path dir = Path.of(System.getProperty("java.io.tmpdir"), "box3dsim", commitTag + "-" + hash);
        Path target = dir.resolve("box3d.dll");
        try {
            if (Files.exists(target) && Files.size(target) == dll.length) {
                return target;
            }
            Files.createDirectories(dir);
            Path temp = Files.createTempFile(dir, "box3d", ".tmp");
            Files.write(temp, dll);
            try {
                Files.move(temp, target, StandardCopyOption.ATOMIC_MOVE);
            } catch (IOException e) {
                // Another process won the race; its copy has the same content.
                Files.deleteIfExists(temp);
                if (!Files.exists(target)) {
                    throw e;
                }
            }
            return target;
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to extract box3d.dll to " + target, e);
        }
    }

    private static String platform() {
        String os = System.getProperty("os.name", "").toLowerCase(Locale.ROOT);
        String arch = System.getProperty("os.arch", "").toLowerCase(Locale.ROOT);
        if (os.startsWith("windows") && (arch.equals("amd64") || arch.equals("x86_64"))) {
            return "windows-x86_64";
        }
        throw new UnsupportedOperationException("box3d-sim v0 ships only windows-x86_64 natives");
    }

    private static String readResource(String name) {
        byte[] bytes = readResourceBytes(name);
        return bytes == null ? null : new String(bytes, StandardCharsets.US_ASCII).trim();
    }

    private static byte[] readResourceBytes(String name) {
        try (InputStream in = NativeLibrary.class.getResourceAsStream(name)) {
            return in == null ? null : in.readAllBytes();
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to read resource " + name, e);
        }
    }

    private static String sha256(byte[] data) {
        try {
            return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(data));
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException(e);
        }
    }
}
