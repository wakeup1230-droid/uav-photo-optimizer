"""Exception hierarchy. Every error raised by the Core derives from UAVPhotoOptimizerError."""


class UAVPhotoOptimizerError(Exception):
    """Base class for all Core errors (stops the whole run)."""


class ConfigError(UAVPhotoOptimizerError):
    """Invalid run configuration."""


class AOIError(UAVPhotoOptimizerError):
    """AOI shapefile missing / ambiguous / unreadable / invalid CRS."""


class PhotoSourceError(UAVPhotoOptimizerError):
    """Photo folder missing or contains no photos."""


class MetadataError(UAVPhotoOptimizerError):
    """Metadata extraction failed as a whole (not a single-photo error)."""


class ExifToolNotFoundError(MetadataError):
    """ExifTool could not be located or executed."""


class ExportError(UAVPhotoOptimizerError):
    """Copying selected photos failed as a whole (e.g. unsafe output folder)."""
