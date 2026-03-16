#!/usr/bin/env python3
"""Moduł korekcji offsetu ruchu dla systemu RoboMVP.

Oblicza korektę trajektorii na podstawie różnicy
między oczekiwaną a wykrytą pozycją markera.
"""


class OffsetCorrector:
    """Kalkulator korekcji offsetu pozycji robota.

    Porównuje oczekiwaną pozę markera z obliczoną
    i zwraca wektor korekcji dx, dy, dz.
    """

    def __init__(self, scale_dx: float = 1.0, scale_dy: float = 1.0,
                 scale_dz: float = 1.0):
        """Inicjalizuje korektor z współczynnikami skalowania.

        Args:
            scale_dx: Współczynnik skalowania korekcji bocznej.
            scale_dy: Współczynnik skalowania korekcji przód/tył.
            scale_dz: Współczynnik skalowania korekcji pionowej.
        """
        self._scale_dx = scale_dx
        self._scale_dy = scale_dy
        self._scale_dz = scale_dz

    def compute_offset(
        self,
        measured_x: float, measured_y: float, measured_z: float,
        expected_x: float = 0.0, expected_y: float = 0.5, expected_z: float = 0.8
    ) -> tuple:
        """Oblicza offset korekcji między pozycją zmierzoną a oczekiwaną.

        Args:
            measured_x: Zmierzona pozycja X markera (metry).
            measured_y: Zmierzona pozycja Y markera (metry).
            measured_z: Zmierzona pozycja Z markera (metry).
            expected_x: Oczekiwana pozycja X markera (metry).
            expected_y: Oczekiwana pozycja Y markera (metry).
            expected_z: Oczekiwana pozycja Z markera (metry).

        Returns:
            Krotka (dx, dy, dz) z wartościami korekcji w metrach.
        """
        raw_dx = expected_x - measured_x
        raw_dy = expected_y - measured_z  # głębokość kamery -> przód robota
        raw_dz = expected_z - measured_y  # oś Y kamery -> pionowa robota

        return self.scale_offset(raw_dx, raw_dy, raw_dz)

    def scale_offset(self, dx: float, dy: float, dz: float) -> tuple:
        """Skaluje już obliczony offset zgodnie z konfiguracją.

        Args:
            dx: Korekcja boczna.
            dy: Korekcja przód/tył.
            dz: Korekcja pionowa.

        Returns:
            Krotka (dx, dy, dz) po zastosowaniu współczynników skali.
        """
        return (
            dx * self._scale_dx,
            dy * self._scale_dy,
            dz * self._scale_dz,
        )

    def apply_offset_to_pose(
        self,
        original_pose: dict,
        dx: float, dy: float, dz: float
    ) -> dict:
        """Stosuje offset do oryginalnej pozy sekwencji ruchu.

        Args:
            original_pose: Słownik z kluczami x, y, z (pozycja).
            dx: Korekcja boczna.
            dy: Korekcja przód/tył.
            dz: Korekcja pionowa.

        Returns:
            Skorygowana poza jako słownik.
        """
        corrected = dict(original_pose)
        corrected['x'] = original_pose.get('x', 0.0) + dx
        corrected['y'] = original_pose.get('y', 0.0) + dy
        corrected['z'] = original_pose.get('z', 0.0) + dz
        return corrected
