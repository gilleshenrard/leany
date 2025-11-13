#!/usr/bin/env python3
import sys
import FreeCAD

def validate_file(filename: str) -> bool:
    """Validate a FreeCAD file. Return True if valid, False otherwise."""
    try:
        doc = FreeCAD.open(filename)
        doc.recompute()
        all_ok = True

        for obj in doc.Objects:
            # Check geometry if shape exists
            if hasattr(obj, "Shape"):
                if not obj.Shape.isValid():
                    print(f"[ERROR] Invalid shape in object: {obj.Name}")
                    all_ok = False

            # Check sketch constraints
            if obj.TypeId == "Sketcher::SketchObject":
                solve_status = obj.solve()
                if solve_status != 0:  # 0 = success
                    print(f"[ERROR] Sketch '{obj.Name}' has solver issues (code {solve_status})")
                    all_ok = False

                if not obj.isFullyConstrained():
                    print(f"[WARNING] Sketch '{obj.Name}' is not fully constrained")

        return all_ok
    except Exception as e:
        print(f"[FATAL] Failed to open {filename}: {e}")
        return False


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: freecadcmd validate_freecad.py <file1.FCStd> [file2.FCStd ...]")
        sys.exit(1)

    status = True
    for f in sys.argv[1:]:
        print(f"Validating: {f}")
        if not validate_file(f):
            status = False

    sys.exit(0 if status else 2)
