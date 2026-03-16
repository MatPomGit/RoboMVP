#!/usr/bin/env python3
"""Narzędzia logowania dla systemu RoboMVP.

Zapewnia znaczniki czasowe (czas od uruchomienia programu)
dla wszystkich komunikatów w konsoli.
"""

import time

_start_time: float = time.monotonic()


def get_elapsed_str() -> str:
    """Zwraca sformatowany czas od uruchomienia programu.

    Returns:
        Ciąg znaków w formacie [HH:MM:SS.mmm].
    """
    elapsed = time.monotonic() - _start_time
    hours = int(elapsed // 3600)
    minutes = int((elapsed % 3600) // 60)
    seconds = elapsed % 60
    return f'[{hours:02d}:{minutes:02d}:{seconds:06.3f}]'


def stamp(message: str) -> str:
    """Poprzedza wiadomość znacznikiem czasu od uruchomienia.

    Args:
        message: Wiadomość do ostemplowania.

    Returns:
        Wiadomość z dołączonym znacznikiem czasowym.
    """
    return f'{get_elapsed_str()} {message}'
