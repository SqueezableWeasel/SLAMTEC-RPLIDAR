
#include <SDL2/SDL.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <string>

#include "sl_lidar.h"

using namespace sl;

// ============================================================
// CONFIGURATION
// ============================================================

constexpr int WINDOW_WIDTH = 900;
constexpr int WINDOW_HEIGHT = 900;

constexpr double PI = 3.14159265358979323846;

// Initial visualization radius
constexpr double INITIAL_RANGE_MM = 2000.0;

// Maximum distance retained from sensor
constexpr double SENSOR_MAX_RANGE_MM = 2000.0;

// Zoom limits
constexpr double MIN_ZOOM_MM = 500.0;
constexpr double MAX_ZOOM_MM = 2000.0;

// Point persistence duration
constexpr uint32_t POINT_LIFETIME_MS = 500;

// Point diameter in pixels
constexpr int POINT_SIZE = 5;

// Serial connection
constexpr const char* SERIAL_PORT = "/dev/ttyUSB0";
constexpr int BAUDRATE = 460800;

// ============================================================
// GLOBALS
// ============================================================

volatile std::sig_atomic_t running = 1;

void signalHandler(int) {
    running = 0;
}

// ============================================================
// POINT STRUCTURE
// ============================================================

struct PersistentPoint {
    double x;
    double y;
    uint8_t quality;
    uint32_t timestamp;
};

// ============================================================
// DRAW CIRCLE
// ============================================================

void drawCircle(
    SDL_Renderer* renderer,
    int cx,
    int cy,
    int radius
) {
    constexpr int segments = 180;

    for (int i = 0; i < segments; ++i) {
        double a1 = 2.0 * PI * i / segments;
        double a2 = 2.0 * PI * (i + 1) / segments;

        SDL_RenderDrawLine(
            renderer,
            cx + static_cast<int>(radius * cos(a1)),
            cy + static_cast<int>(radius * sin(a1)),
            cx + static_cast<int>(radius * cos(a2)),
            cy + static_cast<int>(radius * sin(a2))
        );
    }
}

// ============================================================
// AUTOMATIC GRID SPACING
// ============================================================

double getGridInterval(double maxRangeMM) {
    if (maxRangeMM <= 1000.0)
        return 250.0;

    if (maxRangeMM <= 2500.0)
        return 500.0;

    if (maxRangeMM <= 6000.0)
        return 1000.0;

    return 2000.0;
}

// ============================================================
// DRAW GRID
// ============================================================

void drawGrid(
    SDL_Renderer* renderer,
    int cx,
    int cy,
    double scale,
    double maxRangeMM
) {
    SDL_SetRenderDrawColor(
        renderer, 45, 65, 75, 255
    );

    double interval = getGridInterval(maxRangeMM);

    for (double mm = interval;
         mm <= maxRangeMM;
         mm += interval) {

        int radius = static_cast<int>(mm * scale);

        drawCircle(renderer, cx, cy, radius);
    }

    int radius = static_cast<int>(
        maxRangeMM * scale
    );

    // Horizontal axis
    SDL_RenderDrawLine(
        renderer,
        cx - radius, cy,
        cx + radius, cy
    );

    // Vertical axis
    SDL_RenderDrawLine(
        renderer,
        cx, cy - radius,
        cx, cy + radius
    );
}

// ============================================================
// POINT COLORING
// ============================================================

void getDistanceColor(
    double distance,
    double maxRangeMM,
    uint8_t& r,
    uint8_t& g,
    uint8_t& b
) {
    double t = std::max(
        0.0,
        std::min(distance / maxRangeMM, 1.0)
    );

    // Red -> Yellow -> Green -> Cyan -> Blue
    if (t < 0.25) {
        double f = t / 0.25;
        r = 255;
        g = static_cast<uint8_t>(255 * f);
        b = 60;
    }
    else if (t < 0.5) {
        double f = (t - 0.25) / 0.25;
        r = static_cast<uint8_t>(255 * (1.0 - f));
        g = 255;
        b = 60;
    }
    else if (t < 0.75) {
        double f = (t - 0.5) / 0.25;
        r = 0;
        g = 255;
        b = static_cast<uint8_t>(255 * f);
    }
    else {
        double f = (t - 0.75) / 0.25;
        r = 0;
        g = static_cast<uint8_t>(255 * (1.0 - f));
        b = 255;
    }
}

// ============================================================
// DRAW POINT CLOUD
// ============================================================

void drawPoints(
    SDL_Renderer* renderer,
    const std::vector<PersistentPoint>& points,
    int cx,
    int cy,
    double scale,
    double maxRangeMM
) {
    uint32_t now = SDL_GetTicks();

    for (const auto& p : points) {

        double distance = std::hypot(p.x, p.y);

        // Hide points outside current zoom range
        if (distance > maxRangeMM)
            continue;

        uint8_t r, g, b;

        getDistanceColor(
            distance, maxRangeMM, r, g, b
        );

        // Calculate point age
        uint32_t age = now - p.timestamp;

        if (age >= POINT_LIFETIME_MS)
            continue;

        // Gradual fade
        double fade =
            1.0 -
            static_cast<double>(age) /
            POINT_LIFETIME_MS;

        uint8_t alpha = static_cast<uint8_t>(
            255.0 * fade
        );

        SDL_SetRenderDrawColor(
            renderer, r, g, b, alpha
        );

        int px = cx + static_cast<int>(
            p.x * scale
        );

        int py = cy - static_cast<int>(
            p.y * scale
        );

        SDL_Rect rect = {
            px - POINT_SIZE / 2,
            py - POINT_SIZE / 2,
            POINT_SIZE,
            POINT_SIZE
        };

        SDL_RenderFillRect(renderer, &rect);
    }
}

// ============================================================
// REMOVE EXPIRED POINTS
// ============================================================

void removeExpiredPoints(
    std::vector<PersistentPoint>& points
) {
    uint32_t now = SDL_GetTicks();

    points.erase(
        std::remove_if(
            points.begin(),
            points.end(),
            [now](const PersistentPoint& p) {
                return now - p.timestamp >=
                       POINT_LIFETIME_MS;
            }
        ),
        points.end()
    );
}

// ============================================================
// UPDATE WINDOW TITLE
// ============================================================

void updateWindowTitle(
    SDL_Window* window,
    double maxRangeMM,
    size_t pointCount,
    size_t scanCount
) {
    std::string title =
        "RPLIDAR C1 | Range: " +
        std::to_string(
            static_cast<int>(maxRangeMM)
        ) +
        " mm | Points: " +
        std::to_string(pointCount) +
        " | Scans: " +
        std::to_string(scanCount);

    SDL_SetWindowTitle(window, title.c_str());
}

// ============================================================
// MAIN
// ============================================================

int main() {
    std::signal(SIGINT, signalHandler);

    // --------------------------------------------------------
    // INITIALIZE SDL
    // --------------------------------------------------------

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr
            << "SDL initialization failed: "
            << SDL_GetError() << "\n";

        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "RPLIDAR C1 - 2D Visualization",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH,
        WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );

    if (!window) {
        std::cerr << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(
        window,
        -1,
        SDL_RENDERER_ACCELERATED
    );

    if (!renderer) {
        renderer = SDL_CreateRenderer(
            window,
            -1,
            SDL_RENDERER_SOFTWARE
        );
    }

    if (!renderer) {
        std::cerr << SDL_GetError() << "\n";
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // Enable transparency
    SDL_SetRenderDrawBlendMode(
        renderer,
        SDL_BLENDMODE_BLEND
    );

    // --------------------------------------------------------
    // INITIALIZE RPLIDAR
    // --------------------------------------------------------

    auto channelResult =
        createSerialPortChannel(
            SERIAL_PORT,
            BAUDRATE
        );

    if (!channelResult) {
        std::cerr
            << "Failed to create serial channel.\n";

        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    IChannel* channel = *channelResult;

    auto driverResult = createLidarDriver();

    if (!driverResult) {
        std::cerr
            << "Failed to create LiDAR driver.\n";

        delete channel;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    ILidarDriver* lidar = *driverResult;

    sl_result result = lidar->connect(channel);

    if (SL_IS_FAIL(result)) {
        std::cerr
            << "Failed to connect to RPLIDAR.\n";

        delete lidar;
        delete channel;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    std::cout << "RPLIDAR connected.\n";

    lidar->setMotorSpeed();

    result = lidar->startScan(false, true);

    if (SL_IS_FAIL(result)) {
        std::cerr
            << "Failed to start scanning.\n";

        lidar->setMotorSpeed(0);
        lidar->disconnect();

        delete lidar;
        delete channel;

        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();

        return 1;
    }

    std::cout
        << "Scanning started.\n"
        << "Mouse wheel: Zoom\n"
        << "R: Reset zoom\n"
        << "ESC: Exit\n";

    // --------------------------------------------------------
    // SCAN DATA
    // --------------------------------------------------------

    std::vector<PersistentPoint> points;

    // Prevent excessive vector reallocations
    points.reserve(50000);

    double maxRangeMM = INITIAL_RANGE_MM;

    size_t scanCount = 0;

    uint32_t lastTitleUpdate = 0;

    // --------------------------------------------------------
    // MAIN LOOP
    // --------------------------------------------------------

    while (running) {

        // ----------------------------------------------------
        // SDL EVENTS
        // ----------------------------------------------------

        SDL_Event event;

        while (SDL_PollEvent(&event)) {

            if (event.type == SDL_QUIT) {
                running = 0;
            }

            if (event.type == SDL_KEYDOWN) {

                if (event.key.keysym.sym == SDLK_ESCAPE)
                    running = 0;

                if (event.key.keysym.sym == SDLK_r)
                    maxRangeMM = INITIAL_RANGE_MM;
            }

            if (event.type == SDL_MOUSEWHEEL) {

                if (event.wheel.y > 0)
                    maxRangeMM *= 0.8;

                if (event.wheel.y < 0)
                    maxRangeMM *= 1.25;

                maxRangeMM = std::max(
                    MIN_ZOOM_MM,
                    std::min(
                        maxRangeMM,
                        MAX_ZOOM_MM
                    )
                );
            }
        }

        if (!running)
            break;

        // ----------------------------------------------------
        // ACQUIRE SCAN
        // ----------------------------------------------------

        sl_lidar_response_measurement_node_hq_t
            nodes[8192];

        size_t count = 8192;

        result = lidar->grabScanDataHq(
            nodes,
            count,
            500
        );

        if (SL_IS_OK(result)) {

            lidar->ascendScanData(
                nodes,
                count
            );

            uint32_t timestamp = SDL_GetTicks();

            for (size_t i = 0; i < count; ++i) {

                double angle =
                    nodes[i].angle_z_q14 *
                    90.0 / 16384.0;

                double distance =
                    nodes[i].dist_mm_q2 / 4.0;

                if (distance <= 0.0 ||
                    distance > SENSOR_MAX_RANGE_MM)
                    continue;

                double radians =
                    angle * PI / 180.0;

                PersistentPoint p;

                p.x = distance * sin(radians);
                p.y = distance * cos(radians);

                p.quality = nodes[i].quality;
                p.timestamp = timestamp;

                points.push_back(p);
            }

            ++scanCount;
        }

        // ----------------------------------------------------
        // REMOVE EXPIRED POINTS
        // ----------------------------------------------------

        removeExpiredPoints(points);

        // ----------------------------------------------------
        // WINDOW DIMENSIONS
        // ----------------------------------------------------

        int width, height;

        SDL_GetRendererOutputSize(
            renderer,
            &width,
            &height
        );

        int cx = width / 2;
        int cy = height / 2;

        double radius =
            std::min(width, height) * 0.45;

        double scale =
            radius / maxRangeMM;

        // ----------------------------------------------------
        // CLEAR BACKGROUND
        // ----------------------------------------------------

        SDL_SetRenderDrawColor(
            renderer,
            12, 20, 28, 255
        );

        SDL_RenderClear(renderer);

        // ----------------------------------------------------
        // DRAW GRID
        // ----------------------------------------------------

        drawGrid(
            renderer,
            cx,
            cy,
            scale,
            maxRangeMM
        );

        // ----------------------------------------------------
        // DRAW POINT CLOUD
        // ----------------------------------------------------

        drawPoints(
            renderer,
            points,
            cx,
            cy,
            scale,
            maxRangeMM
        );

        // ----------------------------------------------------
        // DRAW SENSOR CENTER
        // ----------------------------------------------------

        SDL_SetRenderDrawColor(
            renderer,
            255, 180, 50, 255
        );

        SDL_Rect sensor = {
            cx - 4,
            cy - 4,
            8,
            8
        };

        SDL_RenderFillRect(
            renderer,
            &sensor
        );

        // ----------------------------------------------------
        // PRESENT FRAME
        // ----------------------------------------------------

        SDL_RenderPresent(renderer);

        // ----------------------------------------------------
        // UPDATE TITLE
        // ----------------------------------------------------

        uint32_t now = SDL_GetTicks();

        if (now - lastTitleUpdate >= 250) {

            updateWindowTitle(
                window,
                maxRangeMM,
                points.size(),
                scanCount
            );

            lastTitleUpdate = now;
        }
    }

    // --------------------------------------------------------
    // CLEAN SHUTDOWN
    // --------------------------------------------------------

    std::cout << "Stopping RPLIDAR...\n";

    lidar->stop();
    lidar->setMotorSpeed(0);
    lidar->disconnect();

    delete lidar;
    delete channel;

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    std::cout << "Shutdown complete.\n";

    return 0;
}
