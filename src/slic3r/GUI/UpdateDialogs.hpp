#ifndef slic3r_UpdateDialogs_hpp_
#define slic3r_UpdateDialogs_hpp_

#include <string>
#include <unordered_map>
#include <vector>
#include <wx/hyperlink.h>

#include "libslic3r/Semver.hpp"
#include "MsgDialog.hpp"

#include "slic3r/Utils/PresetUpdater.hpp"

class wxBoxSizer;
class wxCheckBox;

namespace Slic3r { namespace GUI {

// App Updater
class AppUpdateAvailableDialog : public MsgDialog {
public:
    AppUpdateAvailableDialog(const Semver& version_current, const Semver& version_online, bool from_user);
    AppUpdateAvailableDialog(AppUpdateAvailableDialog&&) = delete;
    AppUpdateAvailableDialog(const AppUpdateAvailableDialog&) = delete;
    AppUpdateAvailableDialog& operator=(AppUpdateAvailableDialog&&) = delete;
    AppUpdateAvailableDialog& operator=(const AppUpdateAvailableDialog&) = delete;
    virtual ~AppUpdateAvailableDialog();
    
    bool disable_version_check() const;
    static wxSize AUAD_size;
private:
    wxCheckBox* cbox {nullptr};
};

class AppUpdateDownloadDialog : public MsgDialog {
public:
    AppUpdateDownloadDialog(const Semver& version_online, boost::filesystem::path& path);
    AppUpdateDownloadDialog(AppUpdateDownloadDialog&&) = delete;
    AppUpdateDownloadDialog(const AppUpdateDownloadDialog&) = delete;
    AppUpdateDownloadDialog& operator=(AppUpdateDownloadDialog&&) = delete;
    AppUpdateDownloadDialog& operator=(const AppUpdateDownloadDialog&) = delete;
    virtual ~AppUpdateDownloadDialog();
    
    bool run_after_download() const;
    boost::filesystem::path get_download_path() const;
    
private:
    wxStaticText* installing_info;
    wxTextCtrl* txtctrl_path;
    wxString filename;
};


// Confirmation dialog informing about configuration update. Lists updated bundles & their versions.
class MsgUpdateConfig : public MsgDialog
{
public:
    struct Update
    {
        std::string vendor;
        Semver version;
        std::string comment;
        std::string changelog_url;
        std::string new_printers;

        Update(std::string vendor, Semver version, std::string comment, std::string changelog_url, std::string new_printers)
            : vendor(std::move(vendor))
            , version(std::move(version))
            , comment(std::move(comment))
            , changelog_url(std::move(changelog_url))
            , new_printers(std::move(new_printers))
        {}
    };

	// force_before_wizard - indicates that check of updated is forced before ConfigWizard opening
	MsgUpdateConfig(const std::vector<Update> &updates, PresetUpdater::UpdateParams update_params);
	MsgUpdateConfig(MsgUpdateConfig &&) = delete;
	MsgUpdateConfig(const MsgUpdateConfig &) = delete;
	MsgUpdateConfig &operator=(MsgUpdateConfig &&) = delete;
	MsgUpdateConfig &operator=(const MsgUpdateConfig &) = delete;
	~MsgUpdateConfig();
};

// Informs about currently installed bundles not being compatible with the running Slic3r. Asks about action.
class MsgUpdateForced : public MsgDialog
{
public:
    struct Update
    {
        std::string vendor;
        Semver version;
        std::string comment;
        std::string changelog_url;
        std::string new_printers;

        Update(std::string vendor, Semver version, std::string comment, std::string changelog_url, std::string new_printers)
            : vendor(std::move(vendor))
            , version(std::move(version))
            , comment(std::move(comment))
            , changelog_url(std::move(changelog_url))
            , new_printers(std::move(new_printers))
        {}
    };

    MsgUpdateForced(const std::vector<Update>& updates);
    MsgUpdateForced(MsgUpdateForced&&) = delete;
    MsgUpdateForced(const MsgUpdateForced&) = delete;
    MsgUpdateForced& operator=(MsgUpdateForced&&) = delete;
    MsgUpdateForced& operator=(const MsgUpdateForced&) = delete;
    ~MsgUpdateForced();
};

// Informs about currently installed bundles not being compatible with the running Slic3r. Asks about action.
class MsgDataIncompatible : public MsgDialog
{
public:
    // incompats is a map of "vendor name" -> "version restrictions"
    MsgDataIncompatible(const std::unordered_map<std::string, wxString> &incompats);
    MsgDataIncompatible(MsgDataIncompatible &&)      = delete;
    MsgDataIncompatible(const MsgDataIncompatible &) = delete;
    MsgDataIncompatible &operator=(MsgDataIncompatible &&) = delete;
    MsgDataIncompatible &operator=(const MsgDataIncompatible &) = delete;
    ~MsgDataIncompatible();
};

// Informs about a legacy data directory - an update from Slic3r PE < 1.40
class MsgDataLegacy : public MsgDialog
{
public:
    MsgDataLegacy();
    MsgDataLegacy(MsgDataLegacy &&)      = delete;
    MsgDataLegacy(const MsgDataLegacy &) = delete;
    MsgDataLegacy &operator=(MsgDataLegacy &&) = delete;
    MsgDataLegacy &operator=(const MsgDataLegacy &) = delete;
    ~MsgDataLegacy();
};

// Informs about absence of bundles requiring update.
class MsgNoUpdates : public MsgDialog
{
public:
    MsgNoUpdates();
    MsgNoUpdates(MsgNoUpdates &&)      = delete;
    MsgNoUpdates(const MsgNoUpdates &) = delete;
    MsgNoUpdates &operator=(MsgNoUpdates &&) = delete;
    MsgNoUpdates &operator=(const MsgNoUpdates &) = delete;
    ~MsgNoUpdates();
};

class MsgNoAppUpdates : public MsgDialog
{
public:
    MsgNoAppUpdates();
    MsgNoAppUpdates(MsgNoAppUpdates&&) = delete;
    MsgNoAppUpdates(const MsgNoAppUpdates&) = delete;
    MsgNoAppUpdates& operator=(MsgNoUpdates&&) = delete;
    MsgNoAppUpdates& operator=(const MsgNoAppUpdates&) = delete;
    ~MsgNoAppUpdates();
};

}} // namespace Slic3r::GUI

#endif
