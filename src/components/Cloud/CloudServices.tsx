import React, { useState } from 'react';
import { Cloud, Server, Database } from 'lucide-react';

interface CloudService {
  id: string;
  name: string;
  provider: 'aws' | 'gcp' | 'azure';
  status: 'active' | 'inactive' | 'deploying';
}

export const CloudServices: React.FC = () => {
  const [services, setServices] = useState<CloudService[]>([
    { id: '1', name: 'Navigation API', provider: 'aws', status: 'active' },
    { id: '2', name: 'VNC Compute', provider: 'gcp', status: 'active' },
    { id: '3', name: 'Data Storage', provider: 'azure', status: 'inactive' },
  ]);

  return (
    <div className="cloud-services">
      <h3>
        <Cloud size={20} /> Cloud Services
      </h3>
      <div className="services-list">
        {services.map((service) => (
          <div key={service.id} className="service-card">
            <div className="service-header">
              <Server size={16} />
              <span className="service-name">{service.name}</span>
              <span className={`service-status status-${service.status}`}>
                {service.status}
              </span>
            </div>
            <div className="service-provider">
              Provider: {service.provider.toUpperCase()}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

