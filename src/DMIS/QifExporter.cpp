// Copyright (C) 2026 JLS Machine LLC
// SPDX-License-Identifier: GPL-3.0-or-later

#include "dmis/QifExporter.h"
#include <QFile>
#include <QTextStream>
#include <QDateTime>

namespace OpenCMM {

    bool QifExporter::exportToQif(const QString& filePath,
        const QString& partId,
        const std::vector<InspectionResultItem>& results) {
        QFile file(filePath);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            return false;
        }

        QTextStream xml(&file);
        xml.setEncoding(QStringConverter::Utf8);

        const QString timestamp = QDateTime::currentDateTime().toString(Qt::ISODate);

        xml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            << "<QIFDocument xmlns=\"http://resources.qifstandards.org/SCHEMA/QIFv2/2/QIFAll.xsd\"\n"
            << "             xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
            << "             id=\"1\">\n"
            << "  <Version>\n"
            << "    <VersionNumber>2.1</VersionNumber>\n"
            << "  </Version>\n"
            << "  <ApplicationSoftware>\n"
            << "    <Name>OpenCMM Studio</Name> \n"
            << "    <Organization>JLS Machine LLC</Organization>\n"
            << "  </ApplicationSoftware>\n"
            << "  <InspectionDocument>\n"
            << "    <PartSet>\n"
            << "      <Part id=\"1\">\n"
            << "        <Name>" << partId.toHtmlEscaped() << "</Name>\n"
            << "      </Part>\n"
            << "    </PartSet>\n"
            << "    <MeasurementResults id=\"1\">\n"
            << "      <Timestamp>" << timestamp << "</Timestamp>\n"
            << "      <FeatureMeasurementSet>\n";

        int idCounter = 1;
        for (const auto& item : results) {
            xml << "        <FeatureMeasurement id=\"" << idCounter++ << "\">\n"
                << "          <Name>" << item.featureName.toHtmlEscaped() << "</Name>\n"
                << "          <ExternalId>" << item.refId.toHtmlEscaped() << "</ExternalId>\n"
                << "          <ActualValue>" << QString::number(item.actualMm, 'f', 5) << "</ActualValue>\n"
                << "          <NominalValue>" << QString::number(item.nominalMm, 'f', 5) << "</NominalValue>\n"
                << "          <Status>" << (item.passed ? "Pass" : "Fail") << "</Status>\n"
                << "        </FeatureMeasurement>\n";
        }

        xml << "      </FeatureMeasurementSet>\n"
            << "    </MeasurementResults>\n"
            << "  </InspectionDocument>\n"
            << "</QIFDocument>\n";

        file.close();
        return true;
    }

} // namespace OpenCMM