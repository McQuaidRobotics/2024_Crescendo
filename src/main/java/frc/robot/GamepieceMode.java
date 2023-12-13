package frc.robot;

public enum GamepieceMode {
    CONE, CUBE;

    private static GamepieceMode desiredPiece = CONE;
    private static GamepieceMode heldPiece = null;

    public static GamepieceMode getDesiredPiece() {
        return desiredPiece;
    }

    public static void setDesiredPiece(GamepieceMode mode) {
        desiredPiece = mode;
    }

    public static GamepieceMode getHeldPiece() {
        return heldPiece;
    }

    public static void setHeldPiece(GamepieceMode heldPiece) {
        GamepieceMode.heldPiece = heldPiece;
    }
}
