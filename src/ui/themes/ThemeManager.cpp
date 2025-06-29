#include "ThemeManager.hpp"
#include <iostream>

namespace kf::ui::themes {

// Simple global instance without complex singleton pattern
static ThemeManager* g_instance = nullptr;

ThemeManager& ThemeManager::instance() {
    if (!g_instance) {
        g_instance = new ThemeManager();
    }
    return *g_instance;
}

void ThemeManager::initialize() {
    if (initialized_) return;
    
    std::cout << "ThemeManager initialized" << std::endl;
    create_default_themes();
    
    // Set default theme
    if (!themes_.empty()) {
        active_theme_name_ = themes_.begin()->first;
        apply_active_theme();
    }
    
    initialized_ = true;
}

void ThemeManager::cleanup() {
    std::cout << "ThemeManager cleanup" << std::endl;
    themes_.clear();
    active_theme_name_.clear();
    change_callbacks_.clear();
    initialized_ = false;
}

void ThemeManager::register_theme(const std::string& name, std::unique_ptr<PastelTheme> theme) {
    if (theme) {
        themes_[name] = std::move(theme);
        std::cout << "Registered theme: " << name << std::endl;
    }
}

bool ThemeManager::set_active_theme(const std::string& name) {
    auto it = themes_.find(name);
    if (it != themes_.end()) {
        active_theme_name_ = name;
        apply_active_theme();
        notify_theme_change(name);
        return true;
    }
    return false;
}

PastelTheme* ThemeManager::get_active_theme() const {
    auto it = themes_.find(active_theme_name_);
    return (it != themes_.end()) ? it->second.get() : nullptr;
}

PastelTheme* ThemeManager::get_theme(const std::string& name) const {
    auto it = themes_.find(name);
    return (it != themes_.end()) ? it->second.get() : nullptr;
}

std::vector<std::string> ThemeManager::get_theme_names() const {
    std::vector<std::string> names;
    names.reserve(themes_.size());
    for (const auto& pair : themes_) {
        names.push_back(pair.first);
    }
    return names;
}

const std::string& ThemeManager::get_active_theme_name() const {
    return active_theme_name_;
}

void ThemeManager::apply_active_theme() {
    auto* theme = get_active_theme();
    if (theme) {
        theme->apply();
        std::cout << "Applied theme: " << active_theme_name_ << std::endl;
    }
}

void ThemeManager::notify_theme_change(const std::string& theme_name) {
    for (const auto& pair : change_callbacks_) {
        if (pair.second) {
            pair.second(theme_name);
        }
    }
}

void ThemeManager::create_default_themes() {
    // Create default pastel themes
    register_theme("Pastel Light", std::make_unique<PastelTheme>(PastelTheme::Variant::LIGHT));
    register_theme("Pastel Contrast", std::make_unique<PastelTheme>(PastelTheme::Variant::CONTRAST));
    
    std::cout << "Created " << themes_.size() << " default themes" << std::endl;
}

// Global convenience functions
void initialize_theme_manager() {
    ThemeManager::instance().initialize();
}

void cleanup_theme_manager() {
    ThemeManager::instance().cleanup();
}

void apply_theme(const std::string& theme_name) {
    ThemeManager::instance().set_active_theme(theme_name);
}

std::vector<std::string> get_available_themes() {
    return ThemeManager::instance().get_theme_names();
}

ThemeManager& theme_manager() {
    return ThemeManager::instance();
}

} // namespace kf::ui::themes