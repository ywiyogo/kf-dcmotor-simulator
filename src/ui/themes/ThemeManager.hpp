#pragma once

#include "PastelTheme.hpp"
#include <memory>
#include <string>
#include <unordered_map>
#include <functional>

namespace kf::ui::themes {

/**
 * @brief Manages multiple themes and theme switching capabilities
 * 
 * The ThemeManager provides a centralized way to manage different themes,
 * handle theme switching, and persist theme preferences.
 */
class ThemeManager {
public:
    using ThemeChangeCallback = std::function<void(const std::string&)>;

    /**
     * @brief Get singleton instance
     * @return Reference to the theme manager instance
     */
    static ThemeManager& instance();

    /**
     * @brief Initialize the theme manager
     */
    void initialize();

    /**
     * @brief Cleanup theme manager resources
     */
    void cleanup();

    /**
     * @brief Register a new theme
     * @param name Theme name
     * @param theme Theme instance
     */
    void register_theme(const std::string& name, std::unique_ptr<PastelTheme> theme);

    /**
     * @brief Set the active theme
     * @param name Theme name
     * @return true if theme was set successfully
     */
    bool set_active_theme(const std::string& name);

    /**
     * @brief Get the active theme
     * @return Pointer to active theme or nullptr if none set
     */
    PastelTheme* get_active_theme() const;

    /**
     * @brief Get theme by name
     * @param name Theme name
     * @return Pointer to theme or nullptr if not found
     */
    PastelTheme* get_theme(const std::string& name) const;

    /**
     * @brief Get list of available theme names
     * @return Vector of theme names
     */
    std::vector<std::string> get_theme_names() const;

    /**
     * @brief Get active theme name
     * @return Name of active theme
     */
    const std::string& get_active_theme_name() const;

    /**
     * @brief Apply the current active theme
     */
    void apply_active_theme();

    /**
     * @brief Save theme preferences to file
     * @param filename Preferences file path
     * @return true if saved successfully
     */
    bool save_preferences(const std::string& filename) const;

    /**
     * @brief Load theme preferences from file
     * @param filename Preferences file path
     * @return true if loaded successfully
     */
    bool load_preferences(const std::string& filename);

    /**
     * @brief Register callback for theme changes
     * @param name Callback name
     * @param callback Function to call when theme changes
     */
    void register_theme_change_callback(const std::string& name, ThemeChangeCallback callback);

    /**
     * @brief Unregister theme change callback
     * @param name Callback name
     */
    void unregister_theme_change_callback(const std::string& name);

    /**
     * @brief Create default themes
     */
    void create_default_themes();

public:
    ThemeManager() = default;
    ~ThemeManager() = default;
    
private:
    ThemeManager(const ThemeManager&) = delete;
    ThemeManager& operator=(const ThemeManager&) = delete;

    /**
     * @brief Notify all callbacks of theme change
     * @param theme_name Name of new theme
     */
    void notify_theme_change(const std::string& theme_name);

    std::unordered_map<std::string, std::unique_ptr<PastelTheme>> themes_;
    std::string active_theme_name_;
    std::unordered_map<std::string, ThemeChangeCallback> change_callbacks_;
    bool initialized_ = false;
};

// Convenience functions
void initialize_theme_manager();
void cleanup_theme_manager();
ThemeManager& theme_manager();

} // namespace kf::ui::themes