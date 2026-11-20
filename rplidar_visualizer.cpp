
#include <SDL2/SDL.h> // Import the SDL2 window, rendering, and event APIs.
#include <iostream> // Import standard output and error streams.
#include <vector> // Import the resizable container used to store scan points.
#include <algorithm> // Import the min, max, and remove_if algorithms.
#include <cmath> // Import trigonometry and distance functions.
#include <csignal> // Import signal handling and the signal-safe flag type.
#include <cstdint> // Import fixed-width integer types for colors and timestamps.
#include <iomanip> // Import standard stream formatting utilities.
#include <string> // Import strings and number-to-string conversion.

#include "sl_lidar.h" // Import the Slamtec lidar SDK declarations.

using namespace sl; // Make Slamtec SDK types and functions available by their short names.

// ============================================================
// CONFIGURATION
// ============================================================

constexpr int WINDOW_WIDTH = 900; // Set the initial window width to 900 pixels.
constexpr int WINDOW_HEIGHT = 900; // Set the initial window height to 900 pixels.

constexpr double PI = 3.14159265358979323846; // Define pi for angle and circle calculations.

// Initial visualization radius
constexpr double INITIAL_RANGE_MM = 2000.0; // Start with a visible radius of 2 meters.

// Maximum distance retained from sensor
constexpr double SENSOR_MAX_RANGE_MM = 2000.0; // Keep sensor returns within 2 meters of the lidar.

// Zoom limits
constexpr double MIN_ZOOM_MM = 500.0; // Limit the closest zoom to a 500 mm radius.
constexpr double MAX_ZOOM_MM = 2000.0; // Limit the widest zoom to a 2,000 mm radius.

// Point persistence duration
constexpr uint32_t POINT_LIFETIME_MS = 500; // Keep each return visible for up to 500 milliseconds.

// Point diameter in pixels
constexpr int POINT_SIZE = 5; // Draw each return as a five-pixel square.

// Serial connection
constexpr const char* SERIAL_PORT = "/dev/ttyUSB0"; // Select the USB serial device on the Raspberry Pi.
constexpr int BAUDRATE = 460800; // Use the C1 serial speed of 460,800 baud.

// ============================================================
// GLOBALS
// ============================================================

volatile std::sig_atomic_t running = 1; // Keep the display loop active until an exit event or signal.

void signalHandler(int) { // Handle an interrupt signal such as Ctrl+C.
    running = 0; // Request shutdown so the main loop can clean up.
} // End the interrupt handler.

// ============================================================
// POINT STRUCTURE
// ============================================================

struct PersistentPoint { // Store a lidar return for display across several frames.
    double x; // Store the horizontal position in millimeters.
    double y; // Store the vertical position in millimeters.
    uint8_t quality; // Preserve the quality value reported by the SDK.
    uint32_t timestamp; // Store the SDL tick when the scan was processed.
}; // End the scan point type definition.

// ============================================================
// DRAW CIRCLE
// ============================================================

void drawCircle( // Draw a circle by connecting points around its circumference.
    SDL_Renderer* renderer, // Receive the renderer that will draw the circle.
    int cx, // Receive the center's horizontal pixel coordinate.
    int cy, // Receive the center's vertical pixel coordinate.
    int radius // Receive the radius in pixels.
) { // Begin drawing the circle.
    constexpr int segments = 180; // Approximate the circle with 180 straight segments.

    for (int i = 0; i < segments; ++i) { // Draw one segment per iteration.
        double a1 = 2.0 * PI * i / segments; // Find the starting angle of this segment in radians.
        double a2 = 2.0 * PI * (i + 1) / segments; // Find the ending angle of this segment in radians.

        SDL_RenderDrawLine( // Draw a line between the two points on the circle.
            renderer, // Use the caller's SDL renderer.
            cx + static_cast<int>(radius * cos(a1)), // Calculate the starting x coordinate and round toward zero.
            cy + static_cast<int>(radius * sin(a1)), // Calculate the starting y coordinate and round toward zero.
            cx + static_cast<int>(radius * cos(a2)), // Calculate the ending x coordinate and round toward zero.
            cy + static_cast<int>(radius * sin(a2)) // Calculate the ending y coordinate and round toward zero.
        ); // Submit the segment to the renderer.
    } // Finish the loop once all segments are drawn.
} // End the circle drawing function.

// ============================================================
// AUTOMATIC GRID SPACING
// ============================================================

double getGridInterval(double maxRangeMM) { // Choose a grid ring spacing for the visible range.
    if (maxRangeMM <= 1000.0) // Use finer rings for a radius of 1 meter or less.
        return 250.0; // Place the close-range rings 250 mm apart.

    if (maxRangeMM <= 2500.0) // Check whether the visible radius is at most 2.5 meters.
        return 500.0; // Place these rings 500 mm apart.

    if (maxRangeMM <= 6000.0) // Check whether the visible radius is at most 6 meters.
        return 1000.0; // Place these rings 1 meter apart.

    return 2000.0; // Use 2-meter spacing at wider ranges.
} // End the grid spacing function.

// ============================================================
// DRAW GRID
// ============================================================

void drawGrid( // Draw the range rings and center axes.
    SDL_Renderer* renderer, // Receive the renderer that will draw the grid.
    int cx, // Receive the horizontal center of the view in pixels.
    int cy, // Receive the vertical center of the view in pixels.
    double scale, // Receive the number of screen pixels per millimeter.
    double maxRangeMM // Receive the current visible radius in millimeters.
) { // Begin drawing the grid.
    SDL_SetRenderDrawColor( // Select the color shared by the rings and axes.
        renderer, 45, 65, 75, 255 // Use an opaque blue-gray RGBA color.
    ); // Apply the grid color to the renderer.

    double interval = getGridInterval(maxRangeMM); // Choose the physical spacing between adjacent rings.

    for (double mm = interval; // Start at the first nonzero range ring.
         mm <= maxRangeMM; // Stop after the last ring within the visible radius.
         mm += interval) { // Advance one ring spacing after each iteration.

        int radius = static_cast<int>(mm * scale); // Convert this ring's physical radius to pixels.

        drawCircle(renderer, cx, cy, radius); // Draw the ring around the sensor's screen position.
    } // End the loop over range rings.

    int radius = static_cast<int>( // Calculate the maximum grid radius in whole pixels.
        maxRangeMM * scale // Convert the visible range from millimeters to pixels.
    ); // Finish converting the radius to an integer.

    // Horizontal axis
    SDL_RenderDrawLine( // Draw the horizontal axis across the sensor center.
        renderer, // Use the renderer with the current grid color.
        cx - radius, cy, // Start at the left edge of the grid.
        cx + radius, cy // End at the right edge of the grid.
    ); // Submit the horizontal axis to the renderer.

    // Vertical axis
    SDL_RenderDrawLine( // Draw the vertical axis through the sensor center.
        renderer, // Use the same renderer and grid color.
        cx, cy - radius, // Start at the top edge of the grid.
        cx, cy + radius // End at the bottom edge of the grid.
    ); // Submit the vertical axis to the renderer.
} // End the grid drawing function.

// ============================================================
// POINT COLORING
// ============================================================

void getDistanceColor( // Map a distance to a color along a four-part gradient.
    double distance, // Receive the point distance in millimeters.
    double maxRangeMM, // Receive the current visible radius in millimeters.
    uint8_t& r, // Return the red channel through this reference.
    uint8_t& g, // Return the green channel through this reference.
    uint8_t& b // Return the blue channel through this reference.
) { // Begin calculating the color.
    double t = std::max( // Clamp the normalized distance to the interval from zero to one.
        0.0, // Use zero as the minimum.
        std::min(distance / maxRangeMM, 1.0) // Divide by the visible radius and cap the result at one.
    ); // Store the bounded distance fraction in t.

    // Red -> Yellow -> Green -> Cyan -> Blue
    if (t < 0.25) { // Handle points in the nearest quarter of the visible radius.
        double f = t / 0.25; // Map this quarter of the gradient to a fraction from zero to one.
        r = 255; // Keep red at full strength.
        g = static_cast<uint8_t>(255 * f); // Increase green to shift the color toward yellow.
        b = 60; // Keep a small, fixed amount of blue.
    } // End the nearest color band.
    else if (t < 0.5) { // Handle points in the second quarter of the visible radius.
        double f = (t - 0.25) / 0.25; // Measure how far the point is through this color band.
        r = static_cast<uint8_t>(255 * (1.0 - f)); // Decrease red to shift the color toward green.
        g = 255; // Keep green at full strength.
        b = 60; // Keep the same fixed blue level.
    } // End the second color band.
    else if (t < 0.75) { // Handle points in the third quarter of the visible radius.
        double f = (t - 0.5) / 0.25; // Measure how far the point is through this color band.
        r = 0; // Set red to zero.
        g = 255; // Keep green at full strength.
        b = static_cast<uint8_t>(255 * f); // Increase blue to shift the color toward cyan.
    } // End the third color band.
    else { // Handle points in the farthest quarter of the visible radius.
        double f = (t - 0.75) / 0.25; // Measure how far the point is through the final color band.
        r = 0; // Keep red at zero.
        g = static_cast<uint8_t>(255 * (1.0 - f)); // Decrease green to shift the color from cyan toward blue.
        b = 255; // Keep blue at full strength.
    } // End the farthest color band.
} // End the distance-to-color function.

// ============================================================
// DRAW POINT CLOUD
// ============================================================

void drawPoints( // Render the stored lidar returns for the current frame.
    SDL_Renderer* renderer, // Receive the renderer that will draw the points.
    const std::vector<PersistentPoint>& points, // Read the stored points through a reference without copying them.
    int cx, // Receive the sensor's horizontal pixel coordinate.
    int cy, // Receive the sensor's vertical pixel coordinate.
    double scale, // Receive the conversion factor from millimeters to pixels.
    double maxRangeMM // Receive the visible limit for distance and color calculations.
) { // Begin drawing the point cloud.
    uint32_t now = SDL_GetTicks(); // Take a single SDL timestamp for this drawing pass.

    for (const auto& p : points) { // Visit each stored return by reference.

        double distance = std::hypot(p.x, p.y); // Recover the radial distance from the Cartesian coordinates.

        // Hide points outside current zoom range
        if (distance > maxRangeMM) // Check whether the return falls outside the current zoom.
            continue; // Skip this point and continue with the rest.

        uint8_t r, g, b; // Reserve three bytes for the RGB color channels.

        getDistanceColor( // Look up the point's color from its distance.
            distance, maxRangeMM, r, g, b // Pass the range and receive the three channels by reference.
        ); // Finish the color lookup.

        // Calculate point age
        uint32_t age = now - p.timestamp; // Calculate how many milliseconds have elapsed since capture.

        if (age >= POINT_LIFETIME_MS) // Check whether the point has reached its display lifetime.
            continue; // Skip the expired return.

        // Gradual fade
        double fade = // Calculate how much opacity should remain.
            1.0 - // Start at full opacity and subtract the elapsed fraction.
            static_cast<double>(age) / // Convert the age to floating point for fractional division.
            POINT_LIFETIME_MS; // Divide by the full display lifetime.

        uint8_t alpha = static_cast<uint8_t>( // Convert the normalized fade to an 8-bit SDL alpha value.
            255.0 * fade // Scale opacity from zero to 255.
        ); // Finish the alpha conversion.

        SDL_SetRenderDrawColor( // Set the renderer's drawing color for this point.
            renderer, r, g, b, alpha // Apply the distance-based RGB channels and age-based alpha.
        ); // Finish setting the point's drawing color.

        int px = cx + static_cast<int>( // Convert the x offset to an integer screen coordinate.
            p.x * scale // Scale the horizontal offset from millimeters to pixels.
        ); // Finish the x conversion and add the center offset.

        int py = cy - static_cast<int>( // Convert the y offset to screen pixels, where y grows downward.
            p.y * scale // Scale the vertical offset from millimeters to pixels.
        ); // Finish the y conversion and subtract the offset from the center.

        SDL_Rect rect = { // Define the square used to represent this return.
            px - POINT_SIZE / 2, // Offset the left side so the square is centered horizontally.
            py - POINT_SIZE / 2, // Offset the top edge so the square is centered vertically.
            POINT_SIZE, // Set the square's width in pixels.
            POINT_SIZE // Set the square's height in pixels.
        }; // Finish defining the point rectangle.

        SDL_RenderFillRect(renderer, &rect); // Fill the point rectangle with the current RGBA color.
    } // Finish drawing the stored points.
} // End the point drawing function.

// ============================================================
// REMOVE EXPIRED POINTS
// ============================================================

void removeExpiredPoints( // Remove returns that have outlived their display time.
    std::vector<PersistentPoint>& points // Accept the point vector by reference so it can be changed.
) { // Begin pruning expired returns.
    uint32_t now = SDL_GetTicks(); // Use the current SDL tick as the expiration reference.

    points.erase( // Shrink the vector after moving the points to keep to the front.
        std::remove_if( // Move the surviving points forward and find the new logical end.
            points.begin(), // Start the check at the first stored point.
            points.end(), // Stop the check at the end of the vector.
            [now](const PersistentPoint& p) { // Capture the current time and test each point by reference.
                return now - p.timestamp >= // Check whether this point's age has reached the limit.
                       POINT_LIFETIME_MS; // Use the configured lifetime as the cutoff.
            } // End the expiration test.
        ), // Pass the new logical end as the start of the erase range.
        points.end() // Erase through the old end of the vector.
    ); // Finish removing the expired entries.
} // End the point pruning function.

// ============================================================
// UPDATE WINDOW TITLE
// ============================================================

void updateWindowTitle( // Refresh the window title with the current display statistics.
    SDL_Window* window, // Receive the SDL window whose title will change.
    double maxRangeMM, // Receive the current visible radius in millimeters.
    size_t pointCount, // Receive the number of points currently stored.
    size_t scanCount // Receive the number of successfully read scans.
) { // Begin assembling the title.
    std::string title = // Build the title as a single C++ string.
        "RPLIDAR C1 | Range: " + // Start with the sensor name and range label.
        std::to_string( // Convert the displayed range to text.
            static_cast<int>(maxRangeMM) // Drop any fractional millimeters from the title.
        ) + // Continue building the string after inserting the range number.
        " mm | Points: " + // Add the unit and point count label.
        std::to_string(pointCount) + // Append the number of stored points.
        " | Scans: " + // Add the scan count label.
        std::to_string(scanCount); // Append the number of successful scan reads.

    SDL_SetWindowTitle(window, title.c_str()); // Pass SDL a null-terminated version of the finished title.
} // End the title update function.

// ============================================================
// MAIN
// ============================================================

int main() { // Start the visualizer and return a process exit status.
    std::signal(SIGINT, signalHandler); // Register the shutdown handler for Ctrl+C.

    // --------------------------------------------------------
    // INITIALIZE SDL
    // --------------------------------------------------------

    if (SDL_Init(SDL_INIT_VIDEO) != 0) { // Initialize SDL's video subsystem and check for failure.
        std::cerr // Send the diagnostic to standard error.
            << "SDL initialization failed: " // Describe which startup step failed.
            << SDL_GetError() << "\n"; // Append SDL's error details and end the line.

        return 1; // Return a failure status to the shell.
    } // End the SDL initialization error path.

    SDL_Window* window = SDL_CreateWindow( // Create the main SDL window.
        "RPLIDAR C1 - 2D Visualization", // Set the title shown before the first scan update.
        SDL_WINDOWPOS_CENTERED, // Center the window horizontally on the screen.
        SDL_WINDOWPOS_CENTERED, // Center the window vertically on the screen.
        WINDOW_WIDTH, // Use the configured starting width.
        WINDOW_HEIGHT, // Use the configured starting height.
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE // Show the window immediately and allow resizing.
    ); // Store the new window pointer.

    if (!window) { // Handle SDL returning an empty window pointer.
        std::cerr << SDL_GetError() << "\n"; // Print SDL's window creation error.
        SDL_Quit(); // Shut down SDL before exiting.
        return 1; // Return a failure status to the shell.
    } // End the window creation error path.

    SDL_Renderer* renderer = SDL_CreateRenderer( // Try to create a hardware-accelerated renderer.
        window, // Attach the renderer to the visualizer window.
        -1, // Let SDL choose a renderer that supports the requested flags.
        SDL_RENDERER_ACCELERATED // Request hardware acceleration.
    ); // Store the renderer pointer if creation succeeded.

    if (!renderer) { // Fall back when SDL could not create the accelerated renderer.
        renderer = SDL_CreateRenderer( // Try a renderer that draws in software.
            window, // Attach the fallback renderer to the same window.
            -1, // Let SDL select the software rendering driver.
            SDL_RENDERER_SOFTWARE // Request software rendering.
        ); // Store the result of the fallback attempt.
    } // End the fallback renderer setup.

    if (!renderer) { // Handle failure to create either renderer.
        std::cerr << SDL_GetError() << "\n"; // Print the renderer error reported by SDL.
        SDL_DestroyWindow(window); // Release the window created during startup.
        SDL_Quit(); // Shut down the initialized SDL subsystems.
        return 1; // Return a failure status to the shell.
    } // End the renderer creation error path.

    // Enable transparency
    SDL_SetRenderDrawBlendMode( // Configure how newly drawn pixels blend with the frame.
        renderer, // Apply the mode to the selected renderer.
        SDL_BLENDMODE_BLEND // Enable alpha blending so old points can fade.
    ); // Finish configuring the blend mode.

    // --------------------------------------------------------
    // INITIALIZE RPLIDAR
    // --------------------------------------------------------

    auto channelResult = // Keep the factory result so it can be checked before use.
        createSerialPortChannel( // Ask the SDK to create a serial communication channel.
            SERIAL_PORT, // Use the configured device path.
            BAUDRATE // Use the configured serial baud rate.
        ); // Finish creating the channel object.

    if (!channelResult) { // Check whether the SDK failed to create the channel.
        std::cerr // Send the diagnostic to standard error.
            << "Failed to create serial channel.\n"; // Report the serial channel creation failure.

        SDL_DestroyRenderer(renderer); // Release the SDL rendering resources.
        SDL_DestroyWindow(window); // Close the visualization window.
        SDL_Quit(); // Shut down SDL.
        return 1; // Return a failure status to the shell.
    } // End the serial channel creation error path.

    IChannel* channel = *channelResult; // Extract the channel pointer from the successful factory result.

    auto driverResult = createLidarDriver(); // Ask the SDK to create a lidar driver.

    if (!driverResult) { // Check whether driver creation failed.
        std::cerr // Send the diagnostic to standard error.
            << "Failed to create LiDAR driver.\n"; // Report the lidar driver creation failure.

        delete channel; // Delete the channel because the program cannot proceed.
        SDL_DestroyRenderer(renderer); // Release the SDL rendering resources.
        SDL_DestroyWindow(window); // Close the visualization window.
        SDL_Quit(); // Shut down SDL.
        return 1; // Return a failure status to the shell.
    } // End the driver creation error path.

    ILidarDriver* lidar = *driverResult; // Extract the driver pointer from the successful factory result.

    sl_result result = lidar->connect(channel); // Open the serial connection through the channel and save the SDK status.

    if (SL_IS_FAIL(result)) { // Check the SDK status for a connection failure.
        std::cerr // Send the diagnostic to standard error.
            << "Failed to connect to RPLIDAR.\n"; // Report the failure to connect to the lidar.

        delete lidar; // Destroy the driver before releasing its channel.
        delete channel; // Delete the serial channel object.
        SDL_DestroyRenderer(renderer); // Release the SDL rendering resources.
        SDL_DestroyWindow(window); // Close the visualization window.
        SDL_Quit(); // Shut down SDL.
        return 1; // Return a failure status to the shell.
    } // End the lidar connection error path.

    std::cout << "RPLIDAR connected.\n"; // Confirm the SDK opened the lidar connection.

    lidar->setMotorSpeed(); // Request the lidar's default motor speed.

    result = lidar->startScan(false, true); // Start scanning in the device's typical mode without forcing a scan.

    if (SL_IS_FAIL(result)) { // Check whether the SDK failed to start scanning.
        std::cerr // Send the diagnostic to standard error.
            << "Failed to start scanning.\n"; // Report the scan startup failure.

        lidar->setMotorSpeed(0); // Ask the motor to stop because scanning failed.
        lidar->disconnect(); // Close the driver's serial connection.

        delete lidar; // Destroy the disconnected lidar driver.
        delete channel; // Delete the serial channel after the driver is gone.

        SDL_DestroyRenderer(renderer); // Release the SDL rendering resources.
        SDL_DestroyWindow(window); // Close the visualization window.
        SDL_Quit(); // Shut down SDL.

        return 1; // Return a failure status to the shell.
    } // End the scan startup error path.

    std::cout // Print the startup message and controls to standard output.
        << "Scanning started.\n" // Confirm that the scan command succeeded.
        << "Mouse wheel: Zoom\n" // Show how to change the zoom level.
        << "R: Reset zoom\n" // Show the keyboard shortcut for resetting zoom.
        << "ESC: Exit\n"; // Show the keyboard shortcut for exiting.

    // --------------------------------------------------------
    // SCAN DATA
    // --------------------------------------------------------

    std::vector<PersistentPoint> points; // Store recent returns for rendering and fading.

    // Prevent excessive vector reallocations
    points.reserve(50000); // Reserve storage for 50,000 points to reduce reallocations.

    double maxRangeMM = INITIAL_RANGE_MM; // Initialize the visible radius to the default zoom.

    size_t scanCount = 0; // Start the count of successful scans at zero.

    uint32_t lastTitleUpdate = 0; // Initialize the last title update tick for the refresh timer.

    // --------------------------------------------------------
    // MAIN LOOP
    // --------------------------------------------------------

    while (running) { // Repeat event handling, scan reads, and drawing while active.

        // ----------------------------------------------------
        // SDL EVENTS
        // ----------------------------------------------------

        SDL_Event event; // Allocate an event record SDL can fill.

        while (SDL_PollEvent(&event)) { // Read queued SDL events one at a time until the queue is empty.

            if (event.type == SDL_QUIT) { // Detect a request to close the window.
                running = 0; // Request shutdown after processing the queued events.
            } // End the window-close handler.

            if (event.type == SDL_KEYDOWN) { // Check whether the event is a key press.

                if (event.key.keysym.sym == SDLK_ESCAPE) // Look for the Escape key.
                    running = 0; // Request shutdown when Escape is pressed.

                if (event.key.keysym.sym == SDLK_r) // Look for the R key.
                    maxRangeMM = INITIAL_RANGE_MM; // Restore the default zoom level.
            } // End the keyboard event handler.

            if (event.type == SDL_MOUSEWHEEL) { // Check whether the mouse wheel moved.

                if (event.wheel.y > 0) // Check whether the wheel moved upward.
                    maxRangeMM *= 0.8; // Reduce the visible radius by 20 percent to zoom in.

                if (event.wheel.y < 0) // Check whether the wheel moved downward.
                    maxRangeMM *= 1.25; // Increase the visible radius by 25 percent to zoom out.

                maxRangeMM = std::max( // Clamp the visible radius to the configured zoom limits.
                    MIN_ZOOM_MM, // Use the closest supported radius as the lower bound.
                    std::min( // Cap the radius at the widest supported zoom.
                        maxRangeMM, // Use the radius just adjusted by the wheel.
                        MAX_ZOOM_MM // Use the widest supported radius as the upper bound.
                    ) // Finish applying the upper bound.
                ); // Store the clamped radius.
            } // End the mouse-wheel event handler.
        } // Finish processing queued SDL events.

        if (!running) // Check whether an event or signal requested shutdown.
            break; // Leave the main loop before requesting another scan.

        // ----------------------------------------------------
        // ACQUIRE SCAN
        // ----------------------------------------------------

        sl_lidar_response_measurement_node_hq_t // Use the SDK's high-quality measurement record type.
            nodes[8192]; // Allocate 8,192 slots for one scan.

        size_t count = 8192; // Pass the buffer's capacity to the SDK through count.

        result = lidar->grabScanDataHq( // Wait for the SDK to deliver one complete rotation.
            nodes, // Give the SDK the buffer it should fill.
            count, // Let the SDK replace the capacity with the number of received points.
            500 // Wait up to 500 milliseconds for a scan.
        ); // Save the SDK result for the following success check.

        if (SL_IS_OK(result)) { // Process measurements only after a successful read.

            lidar->ascendScanData( // Sort the scan records by angle.
                nodes, // Pass the scan buffer for in-place sorting.
                count // Sort the records returned by this read.
            ); // Finish reordering the scan.

            uint32_t timestamp = SDL_GetTicks(); // Timestamp the scan with the current SDL tick.

            for (size_t i = 0; i < count; ++i) { // Visit every measurement returned by the SDK.

                double angle = // Convert the encoded angle to degrees.
                    nodes[i].angle_z_q14 * // Read the raw angle field from the current measurement.
                    90.0 / 16384.0; // Scale the raw angle so 16,384 units equal 90 degrees.

                double distance = // Convert the encoded distance to millimeters.
                    nodes[i].dist_mm_q2 / 4.0; // Divide the quarter-millimeter distance units by four.

                if (distance <= 0.0 || // Reject missing or nonpositive distance returns.
                    distance > SENSOR_MAX_RANGE_MM) // Also reject distances beyond the configured sensor range.
                    continue; // Skip this measurement and continue through the scan.

                double radians = // Convert the angle to radians for sin and cos.
                    angle * PI / 180.0; // Multiply by pi and divide by 180.

                PersistentPoint p; // Create a display record for the accepted return.

                p.x = distance * sin(radians); // Compute the horizontal offset in millimeters.
                p.y = distance * cos(radians); // Compute the vertical offset in millimeters.

                p.quality = nodes[i].quality; // Copy the return's quality value from the SDK record.
                p.timestamp = timestamp; // Use the scan timestamp to determine when this point fades out.

                points.push_back(p); // Keep the return for rendering in this and subsequent frames.
            } // Finish processing the measurements in this scan.

            ++scanCount; // Count this complete scan as successfully received.
        } // End processing for a successful scan.

        // ----------------------------------------------------
        // REMOVE EXPIRED POINTS
        // ----------------------------------------------------

        removeExpiredPoints(points); // Remove expired returns before drawing the next frame.

        // ----------------------------------------------------
        // WINDOW DIMENSIONS
        // ----------------------------------------------------

        int width, height; // Reserve two integers for the current render output size.

        SDL_GetRendererOutputSize( // Query the renderer's actual output dimensions in pixels.
            renderer, // Use the renderer associated with the window.
            &width, // Let SDL write the output width to width.
            &height // Let SDL write the output height to height.
        ); // Finish reading the output dimensions.

        int cx = width / 2; // Find the horizontal center in pixels.
        int cy = height / 2; // Find the vertical center in pixels.

        double radius = // Calculate a radius that fits a circle within the render area.
            std::min(width, height) * 0.45; // Use 45 percent of the shorter side to leave a margin.

        double scale = // Calculate the current conversion factor from millimeters to pixels.
            radius / maxRangeMM; // Divide the screen radius by the visible physical radius.

        // ----------------------------------------------------
        // CLEAR BACKGROUND
        // ----------------------------------------------------

        SDL_SetRenderDrawColor( // Set the drawing color used to clear the frame.
            renderer, // Apply the color to the active renderer.
            12, 20, 28, 255 // Use a dark, fully opaque RGBA background.
        ); // Finish setting the background color.

        SDL_RenderClear(renderer); // Fill the render target with the background color.

        // ----------------------------------------------------
        // DRAW GRID
        // ----------------------------------------------------

        drawGrid( // Draw the grid over the cleared background.
            renderer, // Draw to the active SDL renderer.
            cx, // Center the grid at the horizontal midpoint.
            cy, // Center the grid at the vertical midpoint.
            scale, // Use the current scale to size the rings.
            maxRangeMM // Use the current zoom to choose ring spacing and extent.
        ); // Finish drawing the grid.

        // ----------------------------------------------------
        // DRAW POINT CLOUD
        // ----------------------------------------------------

        drawPoints( // Draw the recent scan returns over the grid.
            renderer, // Draw to the active SDL renderer.
            points, // Pass the vector of retained scan points.
            cx, // Use the horizontal midpoint as the sensor origin.
            cy, // Use the vertical midpoint as the sensor origin.
            scale, // Convert point offsets using the current display scale.
            maxRangeMM // Hide points outside the visible range and use that range for colors.
        ); // Finish drawing the point cloud.

        // ----------------------------------------------------
        // DRAW SENSOR CENTER
        // ----------------------------------------------------

        SDL_SetRenderDrawColor( // Choose a distinct color for the sensor marker.
            renderer, // Apply the marker color to the active renderer.
            255, 180, 50, 255 // Use an opaque orange RGBA color.
        ); // Finish setting the sensor marker color.

        SDL_Rect sensor = { // Define the square that marks the lidar's position.
            cx - 4, // Place its left edge four pixels left of center.
            cy - 4, // Place its top edge four pixels above center.
            8, // Set the marker width to eight pixels.
            8 // Set the marker height to eight pixels.
        }; // Finish defining the center marker rectangle.

        SDL_RenderFillRect( // Fill the marker on top of the other drawing.
            renderer, // Use the active renderer.
            &sensor // Pass the rectangle defining the marker.
        ); // Finish drawing the center marker.

        // ----------------------------------------------------
        // PRESENT FRAME
        // ----------------------------------------------------

        SDL_RenderPresent(renderer); // Present the completed frame in the SDL window.

        // ----------------------------------------------------
        // UPDATE TITLE
        // ----------------------------------------------------

        uint32_t now = SDL_GetTicks(); // Read the current SDL tick for the title refresh check.

        if (now - lastTitleUpdate >= 250) { // Refresh the title at most once every 250 milliseconds.

            updateWindowTitle( // Rebuild the title with the latest statistics.
                window, // Update the main SDL window.
                maxRangeMM, // Include the currently visible radius.
                points.size(), // Include the number of points still stored.
                scanCount // Include the running count of successful scans.
            ); // Finish updating the title.

            lastTitleUpdate = now; // Save the tick so the next refresh can be timed.
        } // End the title refresh block.
    } // Finish the main loop before releasing resources.

    // --------------------------------------------------------
    // CLEAN SHUTDOWN
    // --------------------------------------------------------

    std::cout << "Stopping RPLIDAR...\n"; // Tell the user the lidar is shutting down.

    lidar->stop(); // Ask the lidar to stop sending scans.
    lidar->setMotorSpeed(0); // Request a motor speed of zero.
    lidar->disconnect(); // Close the driver's connection to the serial channel.

    delete lidar; // Destroy the lidar driver.
    delete channel; // Destroy the serial channel after its driver is gone.

    SDL_DestroyRenderer(renderer); // Free the renderer before destroying its window.
    SDL_DestroyWindow(window); // Close and free the SDL window.
    SDL_Quit(); // Shut down the SDL subsystems.

    std::cout << "Shutdown complete.\n"; // Confirm that shutdown completed.

    return 0; // Return a successful exit status to the shell.
} // End the main function.
